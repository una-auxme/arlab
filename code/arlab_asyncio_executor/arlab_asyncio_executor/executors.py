# This file is heavily based on the implementations in rclpy.executors and rclpy.task
# and adapted to work with asyncio.

import concurrent.futures
from typing import Optional, Union, Callable, Coroutine
import inspect
import concurrent
import asyncio

from rclpy.executors import (
    Executor,
    TimeoutException,
    ExternalShutdownException,
    ShutdownException,
    ConditionReachedException,
    TimeoutObject,
    WaitableEntityType,
)
from rclpy.context import Context
from rclpy.exceptions import InvalidHandle
from rclpy.task import Future as RosFuture
from rclpy.task import Task as RosTask
from rclpy.node import Node


class AsyncIORosTask(RosTask):
    def __init__(self, handler, asyncio_loop, args=None, kwargs=None, executor=None):
        super().__init__(handler=handler, args=args, kwargs=kwargs, executor=executor)

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

    def __init__(self, asyncio_loop, *, context: Optional[Context] = None) -> None:
        super().__init__(context=context)

        self._asyncio_loop = asyncio_loop
        self._asyncio_running_tasks = []

    def create_task(
        self, callback: Union[Callable, Coroutine], *args, **kwargs
    ) -> RosTask:
        task = AsyncIORosTask(callback, self._asyncio_loop, args, kwargs, executor=self)
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
            (entity, self._guard, self._is_shutdown, self._work_tracker),
            executor=self,
        )
        with self._tasks_lock:
            self._tasks.append((task, entity, node))
        return task

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
            self._asyncio_running_tasks.append(task)
            # make a copy of the list that we iterate over while modifying it
            # (https://stackoverflow.com/q/1207406/3753684)
            for task in self._asyncio_running_tasks[:]:
                if task.done():
                    self._asyncio_running_tasks.remove(task)
                    task.result()  # raise any exceptions

    def spin_once(self, timeout_sec: Optional[float] = None) -> None:
        self._spin_once_impl(timeout_sec)

    def spin_once_until_future_complete(
        self,
        future: RosFuture,
        timeout_sec: Optional[Union[float, TimeoutObject]] = None,
    ) -> None:
        future.add_done_callback(lambda x: self.wake())
        self._spin_once_impl(timeout_sec, future.done)
