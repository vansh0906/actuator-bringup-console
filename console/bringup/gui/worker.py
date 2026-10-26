"""Runs long operations (calibration, bring-up tests, parameter dumps) off
the GUI thread. The GUI polls `poll()` from its timer, so every callback
runs on the GUI thread and no Qt object is touched from a worker."""

from __future__ import annotations

import queue
import threading
import traceback
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Any, Callable


class TaskRunner:
    def __init__(self):
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="gui-task")
        self._pending: list[tuple[str, Future, Callable[[Any], None] | None, Callable[[str], None] | None]] = []
        self.messages: queue.Queue[str] = queue.Queue()
        self.stop_requested = threading.Event()
        self.current: str = ""

    @property
    def busy(self) -> bool:
        return any(not f.done() for _, f, _, _ in self._pending)

    def log(self, text: str) -> None:
        """Thread-safe: workers call this to stream progress to the GUI."""
        self.messages.put(text)

    def run(self, name: str, fn: Callable, *args, on_done: Callable[[Any], None] | None = None,
            on_error: Callable[[str], None] | None = None, **kwargs) -> bool:
        if self.busy:
            self.log(f"busy with {self.current}: wait for it to finish")
            return False
        self.stop_requested.clear()
        self.current = name
        future = self._pool.submit(fn, *args, **kwargs)
        self._pending.append((name, future, on_done, on_error))
        return True

    def poll(self) -> list[str]:
        """Call from the GUI thread: fires completion callbacks, returns new log lines."""
        lines = []
        while True:
            try:
                lines.append(self.messages.get_nowait())
            except queue.Empty:
                break
        still = []
        for name, future, on_done, on_error in self._pending:
            if not future.done():
                still.append((name, future, on_done, on_error))
                continue
            exc = future.exception()
            if exc is None:
                if on_done:
                    on_done(future.result())
            else:
                text = "".join(traceback.format_exception_only(type(exc), exc)).strip()
                lines.append(f"{name} failed: {text}")
                if on_error:
                    on_error(text)
        self._pending = still
        if not still:
            self.current = ""
        return lines

    def shutdown(self) -> None:
        self.stop_requested.set()
        self._pool.shutdown(wait=False, cancel_futures=True)
