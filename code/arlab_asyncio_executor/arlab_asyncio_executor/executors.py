# Copyright 2017 Open Source Robotics Foundation, Inc.
# Copyright 2025 Peter Viechter, UNA-AuxMe: Adaptations for asyncio compatibility
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This file is heavily based on the implementations in rclpy.executors and rclpy.task
# and adapted to work with asyncio.

"""Contains a ROS executor that is asyncio compatible.

Classes:
- AsyncIORosTask(RosTask): A RosTask that runs in an asyncio event loop.
- AsyncIOExecutor(Executor): ROS executor that has been integrated
    with asyncio to improve throughput

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
"""

import asyncio
import concurrent
import concurrent.futures
import inspect
import queue
import signal
import sys
import threading
from queue import Queue
from typing import Callable, Coroutine, Optional, Union

from rclpy.context import Context
from rclpy.exceptions import InvalidHandle
from rclpy.executors import (
    ConditionReachedException,
    Executor,
    ExternalShutdownException,
    ShutdownException,
    TaskData,
    TimeoutException,
    TimeoutObject,
    WaitableEntityType,
)
from rclpy.node import Node
from rclpy.task import Future as RosFuture
from rclpy.task import Task as RosTask


class AsyncIORosTask(RosTask):
    """A RosTask that runs in an asyncio event loop.

    This RosTask has been modified to put itself in an asyncio event loop
    (if handler is a coroutine)
    """

    def __init__(
        self,
        handler,
        asyncio_loop,
        exception_queue: Queue,
        args=None,
        kwargs=None,
        executor=None,
    ):
        """Create a new ros task

        Args:
            handler (Callable or Coroutine): Main handler function
                to be called when the task is run.
            asyncio_loop (_type_): Asyncio event loop to run the handler on
            exception_queue (Queue): Queue to put exceptions in.
                They are read and output by the main executor
            args (_type_, optional): _description_. Defaults to None.
            kwargs (_type_, optional): _description_. Defaults to None.
            executor (_type_, optional): _description_. Defaults to None.
        """
        super().__init__(handler=handler, args=args, kwargs=kwargs, executor=executor)

        self._exception_queue = exception_queue
        self._asyncio_loop = asyncio_loop
        self._asyncio_future: Optional[concurrent.futures.Future] = None

    def __call__(self):
        """Run or resume a task.

        This attempts to execute a handler.
        If the handler is a coroutine it will run it on the self._asyncio_loop.

        The return value of the handler is stored as the task result.
        """
        if (
            not self._pending()
            or self._executing
            or not self._task_lock.acquire(blocking=False)
        ):
            return

        if not self._pending():
            return

        if inspect.iscoroutine(self._handler):
            # Execute a coroutine with asyncio
            # https://docs.python.org/3/library/asyncio-dev.html#asyncio-multithreading
            async def wrapped_handler():
                """Wraps the handler with exception handling"""
                try:
                    result = await self._handler
                    self.set_result(result)
                except BaseException as e:
                    self.set_exception(e)
                    self._exception_queue.put_nowait(e)
                finally:
                    self._complete_task()
                    self._executing = False
                    self._task_lock.release()

            if not self._asyncio_future:
                # Only start the task if it has not already been started
                self._executing = True
                self._asyncio_future = asyncio.run_coroutine_threadsafe(
                    wrapped_handler(), self._asyncio_loop
                )
        else:
            # Execute a normal function
            self._executing = True
            try:
                self.set_result(self._handler(*self._args, **self._kwargs))
            except BaseException as e:
                self.set_exception(e)
                self._exception_queue.put_nowait(e)
            finally:
                self._complete_task()
                self._executing = False
                self._task_lock.release()


class AsyncIOExecutor(Executor):
    """ROS executor that has been integrated with asyncio to improve throughput

    The default and MultiThreaded ROS executors
    do not support asyncio based futures.
    This Executor runs async callbacks inside asyncio.
    -> Makes using asyncio in ros callbacks possible.

    Usage recommendation: Nodes that make heavy, concurrent use of I/O.

    Based on the rclpy MultiThreadedExecutor
    """

    def __init__(
        self, async_init: Coroutine, *, context: Optional[Context] = None
    ) -> None:
        """Initializes the executor

        Args:
            async_init (Coroutine): Coroutine that is run right after
                the asyncio event loop is started up
            context (Optional[Context], optional): Executor context. Defaults to None.
        """
        super().__init__(context=context)

        self._spin_thread = None
        """Thread spinning the executor"""
        self._exception_queue = Queue()
        """Queue containing raised exceptions inside any ros tasks"""

        loop_setup_event = threading.Event()
        """Fired when the asyncio event loop is set up and async_init is done"""
        self._shutdown_event = threading.Event()
        """Fired when the executor is shutting down"""
        self._asyncio_loop = None
        """The asyncio event loop created by this executor"""

        def asyncio_thread():
            async def asyncio_wait():
                """Main asyncio thread function

                1. Runs async_init
                2. Sets self._asyncio_loop
                3. Signals that the asyncio event loop is ready
                    This makes the main thread finish the __init__
                4. Waits on the self._shutdown_event
                """
                try:
                    await async_init
                    self._asyncio_loop = asyncio.get_running_loop()
                    loop_setup_event.set()
                    await asyncio.to_thread(self._shutdown_event.wait)
                except BaseException as e:
                    self._exception_queue.put_nowait(e)
                finally:
                    all_tasks = asyncio.all_tasks()
                    all_tasks.remove(asyncio.current_task())
                    for task in all_tasks:
                        task.cancel()
                    print("All asyncio tasks cancelled.")

                    # Make sure we don't block if an exception was raised
                    # before the loop was setup
                    loop_setup_event.set()

            asyncio.run(asyncio_wait())

        self._asyncio_thread = threading.Thread(target=asyncio_thread, daemon=True)
        """Thread running the asyncio event loop"""
        self._asyncio_thread.start()
        loop_setup_event.wait()
        # Check for exceptions that happened during loop setup
        try:
            e = self._exception_queue.get_nowait()
            # Cleanup
            self.shutdown()
            raise e from e
        except queue.Empty:
            pass

    def create_task(
        self, callback: Union[Callable, Coroutine], *args, **kwargs
    ) -> RosTask:
        # This function is the same as super().create_task,
        # it just creates an AsyncIORosTask instead.
        task = AsyncIORosTask(
            callback,
            self._asyncio_loop,
            self._exception_queue,
            args,
            kwargs,
            executor=self,
        )
        with self._tasks_lock:
            self._pending_tasks[task] = TaskData()
        self._call_task_in_next_spin(task)
        return task

    def _make_handler(
        self,
        entity: WaitableEntityType,
        node: Node,
        take_from_wait_list: Callable,
    ) -> RosTask:
        # This function is the same as super()._make_handler,
        # it just creates an AsyncIORosTask instead.
        entity._executor_event = True

        async def handler(entity, gc, is_shutdown, work_tracker):
            if is_shutdown or not entity.callback_group.beginning_execution(entity):
                entity._executor_event = False
                gc.trigger()
                return
            with work_tracker:
                call_coroutine = take_from_wait_list(entity)

                entity._executor_event = False
                gc.trigger()

                try:
                    if call_coroutine is not None:
                        await call_coroutine()
                finally:
                    entity.callback_group.ending_execution(entity)
                    try:
                        gc.trigger()
                    except InvalidHandle:
                        pass

        task = AsyncIORosTask(
            handler,
            self._asyncio_loop,
            self._exception_queue,
            (entity, self._guard, self._is_shutdown, self._work_tracker),
            executor=self,
        )
        with self._tasks_lock:
            self._pending_tasks[task] = TaskData(source_entity=entity, source_node=node)
        return task

    def start_spin_thread(self):
        """Starts the thread that spins the executor

        This means it runs the super().spin function which
        runs self.spin_one() in a loop.
        """
        if self._spin_thread is not None:
            return

        super_spin_fn = super().spin

        def spin_thread():
            try:
                super_spin_fn()
            except BaseException as e:
                self._exception_queue.put_nowait(e)

        self._spin_thread = threading.Thread(target=spin_thread, daemon=True)
        self._spin_thread.start()

    def spin(self):
        """Execute callbacks until shutdown.

        New spin function that starts the separate spin thread and
        then waits for any exceptions on the main thread."""
        self.start_spin_thread()
        self.wait_for_exception()

    def _spin_once_impl(
        self,
        timeout_sec: Optional[Union[float, TimeoutObject]] = None,
        wait_condition: Callable[[], bool] = lambda: False,
    ) -> None:
        # Based on SingleThreadedExecutor._spin_once_impl.
        # Only any task exception handling was removed here because
        # exceptions are handled by the main thread not the spin_thread.
        try:
            task, entity, node = self.wait_for_ready_callbacks(
                timeout_sec, None, wait_condition
            )
        except ExternalShutdownException:
            pass
        except ShutdownException:
            pass
        except TimeoutException:
            pass
        except ConditionReachedException:
            pass
        else:
            task()

    def spin_once(self, timeout_sec: Optional[float] = None) -> None:
        # Exactly matches SingleThreadedExecutor.spin_once
        self._spin_once_impl(timeout_sec)

    def spin_once_until_future_complete(
        self,
        future: RosFuture,
        timeout_sec: Optional[Union[float, TimeoutObject]] = None,
    ) -> None:
        # Exactly matches SingleThreadedExecutor.spin_once_until_future_complete
        future.add_done_callback(lambda x: self.wake())
        self._spin_once_impl(timeout_sec, future.done)

    def wait_for_exception(self, shutdown_timeout: Optional[float] = 5.0):
        """Blocks until an exception occurs or an exit signal is received

        Raises:
            e: The exception that occurred
        """

        def signal_handler(signum, frame):
            signal_name = signal.Signals(signum).name
            print(f"Received signal {signal_name}. Shutting down.")
            self.shutdown(shutdown_timeout)
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        e = self._exception_queue.get()
        if e is not None:
            # Cleanup
            self.shutdown(shutdown_timeout)
            raise e from e

    def shutdown(self, timeout_sec: Optional[float] = 5.0) -> bool:
        """Signals the self._shutdown_event and joins all threads

        Args:
            timeout_sec (Optional[float], optional):
                Maximum time to wait for a thread to finish. Defaults to 5.0.

        Returns:
            bool: True if all outstanding callbacks finished executing,
                or False if the timeout expires before all outstanding work is done.
        """
        result = super().shutdown(timeout_sec)
        self._shutdown_event.set()
        print("Waiting for asyncio thread to finish...")
        self._asyncio_thread.join(timeout=timeout_sec)
        result = result and not self._asyncio_thread.is_alive()
        if self._spin_thread is not None:
            print("Waiting for spin thread to finish...")
            self._spin_thread.join(timeout=timeout_sec)
            result = result and not self._spin_thread.is_alive()
        return result
