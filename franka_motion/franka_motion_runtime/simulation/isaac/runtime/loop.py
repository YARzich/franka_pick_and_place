"""
Simulation loop helpers for the Isaac Sim runtime.

Responsibility:
- Own signal-aware loop execution and bounded shutdown cleanup.

Does not own:
- Scene construction, graph creation or launch process management.
"""

from __future__ import annotations

import signal
import threading
from typing import Callable

from .lifecycle import close_world_and_app


def install_stop_signal_handlers(stop_event: threading.Event) -> None:
    """Map SIGINT and SIGTERM to the shared simulation stop event."""

    def _handle_signal(_signum: int, _frame) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)


def run_simulation_loop(
    world,
    simulation_app,
    *,
    headless: bool,
    time_steps: int,
    stop_event: threading.Event | None = None,
    before_step: Callable[[], None] | None = None,
    after_step: Callable[[], None] | None = None,
) -> None:
    """Step the Isaac world until cancellation, app closure or step limit."""
    event = threading.Event() if stop_event is None else stop_event
    render_flag = not headless
    steps_remaining = time_steps if time_steps > 0 else None
    world.play()

    while not event.is_set() and simulation_app.is_running():
        if before_step is not None:
            before_step()
        world.step(render=render_flag)
        if after_step is not None:
            after_step()
        if headless:
            simulation_app.update()

        if steps_remaining is not None:
            steps_remaining -= 1
            if steps_remaining <= 0:
                break


__all__ = [
    "close_world_and_app",
    "install_stop_signal_handlers",
    "run_simulation_loop",
]
