# This file is heavily based on the implementations in rclpy.executors and rclpy.task
# and adapted to work with asyncio.

import asyncio
import concurrent
import concurrent.futures
import inspect
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
    TimeoutException,
    TimeoutObject,
    WaitableEntityType,
)
from rclpy.node import Node
from rclpy.task import Future as RosFuture
from rclpy.task import Task as RosTask


class AsyncIORosTask(RosTask):
    def __init__(
        self,
        handler,
        asyncio_loop,
        exception_queue: Queue,
        args=None,
        kwargs=None,
        executor=None,
    ):
        super().__init__(handler=handler, args=args, kwargs=kwargs, executor=executor)

        self._exception_queue = exception_queue
        self._asyncio_loop = asyncio_loop
        self._asyncio_future: Optional[concurrent.futures.Future] = None

    def __call__(self):
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
                self._executing = True
                self._asyncio_future = asyncio.run_coroutine_threadsafe(
                    wrapped_handler(), self._asyncio_loop
                )
        else:
            try:
                # Execute a normal function
                self._executing = True
                try:
                    self.set_result(self._handler(*self._args, **self._kwargs))
                except Exception as e:
                    self.set_exception(e)
                self._complete_task()
                self._executing = False

            finally:
                self._task_lock.release()


class AsyncIOExecutor(Executor):
    # Based on the rclpy MultiThreadedExecutor

    def __init__(
        self, async_init: Coroutine, *, context: Optional[Context] = None
    ) -> None:
        super().__init__(context=context)

        self._spin_thread = None
        self._exception_queue = Queue()

        loop_setup_event = threading.Event()
        self._asyncio_shutdown_event = asyncio.Event()
        self._asyncio_loop = None

        def asyncio_thread():
            async def asyncio_wait():
                try:
                    await async_init
                    self._asyncio_loop = asyncio.get_running_loop()
                    loop_setup_event.set()
                    await self._asyncio_shutdown_event.wait()
                except BaseException as e:
                    self._exception_queue.put_nowait(e)
                finally:
                    for task in asyncio.all_tasks():
                        task.cancel()

            asyncio.run(asyncio_wait())

        self._asyncio_thread = threading.Thread(target=asyncio_thread, daemon=True)
        self._asyncio_thread.start()
        loop_setup_event.wait()

    def create_task(
        self, callback: Union[Callable, Coroutine], *args, **kwargs
    ) -> RosTask:
        task = AsyncIORosTask(
            callback,
            self._asyncio_loop,
            self._exception_queue,
            args,
            kwargs,
            executor=self,
        )
        with self._tasks_lock:
            self._tasks.append((task, None, None))
            self._guard.trigger()
        # Task inherits from Future
        return task

    def _make_handler(
        self,
        entity: WaitableEntityType,
        node: Node,
        take_from_wait_list: Callable,
    ) -> RosTask:
        # Heavily based on rclpy.executors.Executor._make_handler

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
            self._tasks.append((task, entity, node))
        return task

    def start_spin_thread(self):
        if self._spin_thread is not None:
            return

        self._spin_thread = threading.Thread(target=super().spin, daemon=True)
        self._spin_thread.start()

    def spin(self):
        self.start_spin_thread()
        self.wait_for_exception()

    def _spin_once_impl(
        self,
        timeout_sec: Optional[Union[float, TimeoutObject]] = None,
        wait_condition: Callable[[], bool] = lambda: False,
    ) -> None:
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
        self._spin_once_impl(timeout_sec)

    def spin_once_until_future_complete(
        self,
        future: RosFuture,
        timeout_sec: Optional[Union[float, TimeoutObject]] = None,
    ) -> None:
        future.add_done_callback(lambda x: self.wake())
        self._spin_once_impl(timeout_sec, future.done)

    def wait_for_exception(self, shutdown_timeout: Optional[float] = 1.0):
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
            raise e

    def shutdown(self, timeout_sec=None):
        result = super().shutdown(timeout_sec)
        self._asyncio_shutdown_event.set()
        self._asyncio_thread.join(timeout=timeout_sec)
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=timeout_sec)
        return result
