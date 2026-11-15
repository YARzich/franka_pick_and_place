"""
Isaac Sim runtime orchestration for Franka ``ros2_control`` scenarios.

Responsibility:
- Bootstrap Isaac Sim Python without mixing system and Isaac ROS 2 libraries.
- Orchestrate the Franka world, optional table/cube/stereo camera scene and ROS bridge graphs.
- Run the simulation loop with signal-aware shutdown for launch-managed scenarios.

Does not own:
- ROS2 launch composition, controller spawning or MoveIt/curobo planner startup.
- CLI bootstrapping and package-path discovery performed by the installed script.
- Perception backend inference logic consuming the published camera topics.
"""

from __future__ import annotations

from dataclasses import replace
import sys
import threading

from .runtime.config import (
    IsaacRunnerConfig,
    parse_runner_config,
    validate_runner_paths,
)
from .runtime.environment import (
    bootstrap_isaacsim_python,
    cleanup_previous_minidumps,
    disable_warp,
    ensure_ros_bridge_environment,
    reexec_with_clean_env,
    resolve_isaac_root,
)
from .runtime.lifecycle import (
    IsaacRuntimeOwner,
    run_with_runtime_owner,
)
from .runtime.loop import (
    install_stop_signal_handlers,
    run_simulation_loop,
)
from .runtime.physical_grasp import create_pick_cube_attachment_service
from .runtime.ros_graph import (
    create_joint_ros_graph,
    create_stereo_ros_graph,
)
from .runtime.scene import (
    add_franka_robot,
    add_pick_cube,
    add_scene_reference,
    create_world,
    initialize_robot,
    load_robot_articulation,
)
from .runtime.stereo import (
    create_stereo_camera_prims,
    create_stereo_visual_model,
)


class IsaacStartupCancelled(RuntimeError):
    """Internal control flow used after a launch-managed stop request."""


def _enable_runtime_extensions() -> None:
    from isaacsim.core.utils import extensions as isaac_ext

    for name in (
        "isaacsim.ros2.bridge",
        "isaacsim.ros2.urdf",
        "isaacsim.core.api",
    ):
        isaac_ext.enable_extension(name)


def main() -> None:
    """Validate configuration, isolate the environment and run Isaac Sim."""
    config = parse_runner_config()
    config = replace(config, install_path=resolve_isaac_root(config.install_path))
    isaac_root = config.install_path
    validate_runner_paths(config)
    reexec_with_clean_env(isaac_root, config.ros_distro)
    stop_event = threading.Event()
    install_stop_signal_handlers(stop_event)
    try:
        run_with_runtime_owner(
            lambda runtime: _run_configured_simulation(
                config,
                runtime,
                stop_event,
            )
        )
    except IsaacStartupCancelled:
        return


def _run_configured_simulation(
    config: IsaacRunnerConfig,
    runtime: IsaacRuntimeOwner,
    stop_event: threading.Event,
) -> None:
    """Construct and run one simulation while registering owned resources."""
    _raise_if_stopping(stop_event)
    isaac_root = config.install_path
    bootstrap_isaacsim_python(isaac_root)
    ensure_ros_bridge_environment(isaac_root, config.ros_distro)
    disable_warp()
    cleanup_previous_minidumps(isaac_root)
    _raise_if_stopping(stop_event)

    from isaacsim import SimulationApp

    sim_app_config = {"headless": config.headless}
    if config.experience:
        runtime.simulation_app = SimulationApp(
            sim_app_config,
            experience=config.experience,
        )
    else:
        runtime.simulation_app = SimulationApp(sim_app_config)
    _raise_if_stopping(stop_event)

    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
    runtime.simulation_app.update()
    _raise_if_stopping(stop_event)

    import omni.graph.core as og  # noqa: WPS433 (import after SimulationApp)
    import usdrt.Sdf  # noqa: WPS433

    _enable_runtime_extensions()
    from isaacsim.core.api import World  # noqa: WPS433

    runtime.world_cls = World
    runtime.world = create_world(no_default_ground=config.no_default_ground)
    _raise_if_stopping(stop_event)
    _run_world(
        config,
        runtime.world,
        runtime.simulation_app,
        og,
        usdrt.Sdf.Path,
        stop_event,
    )


def _run_world(
    config: IsaacRunnerConfig,
    world,
    simulation_app,
    og,
    path_factory,
    stop_event: threading.Event,
) -> None:
    """Assemble scene and graphs, then enter the signal-aware simulation loop."""
    _raise_if_stopping(stop_event)
    stage = world.scene.stage
    articulation_path = load_robot_articulation(stage, config)
    if not articulation_path:
        raise RuntimeError("Путь артикуляции не определён")

    add_scene_reference(config.scene_usd)
    pick_cube = add_pick_cube(world, config.cube)
    _raise_if_stopping(stop_event)

    stereo_paths = None
    if config.stereo.enabled:
        stereo_paths = create_stereo_camera_prims(
            stage,
            config.stereo,
            parent_path="/World",
        )
        create_stereo_visual_model(stage, stereo_paths.rig_root_path, config.stereo)
    _raise_if_stopping(stop_event)

    franka = add_franka_robot(world, articulation_path)
    simulation_app.update()
    if not initialize_robot(world, franka, should_stop=stop_event.is_set):
        _raise_if_stopping(stop_event)
        raise RuntimeError(
            "Articulation для Franka не инициализировалась: "
            f"{articulation_path}"
        )
    _raise_if_stopping(stop_event)

    try:
        graph_path = create_joint_ros_graph(
            og,
            path_factory,
            articulation_path=articulation_path,
            command_topic=config.command_topic,
            state_topic=config.state_topic,
        )
    except Exception as exc:
        raise RuntimeError("Не удалось создать граф ROS2 моста") from exc
    og.Controller.evaluate_sync(graph_path)
    _raise_if_stopping(stop_event)

    if stereo_paths is not None:
        try:
            stereo_graph_path = create_stereo_ros_graph(
                og,
                path_factory,
                stereo=config.stereo,
                left_camera_path=stereo_paths.left_camera_path,
                right_camera_path=stereo_paths.right_camera_path,
            )
        except Exception as exc:
            raise RuntimeError(
                "Не удалось создать граф stereo-камеры"
            ) from exc
        og.Controller.evaluate_sync(stereo_graph_path)
        _raise_if_stopping(stop_event)

    physical_attachment = None
    try:
        if pick_cube is not None:
            physical_attachment = create_pick_cube_attachment_service(
                stage,
                articulation_path=articulation_path,
                cube_path=config.cube.prim,
            )
        run_simulation_loop(
            world,
            simulation_app,
            headless=config.headless,
            time_steps=config.time_steps,
            stop_event=stop_event,
            before_step=(
                physical_attachment.before_step
                if physical_attachment is not None
                else None
            ),
            after_step=(
                physical_attachment.after_step
                if physical_attachment is not None
                else None
            ),
        )
    finally:
        if physical_attachment is not None:
            _close_physical_attachment(physical_attachment)


def _raise_if_stopping(stop_event: threading.Event) -> None:
    """Abort startup at deterministic cleanup checkpoints after SIGINT/TERM."""
    if stop_event.is_set():
        raise IsaacStartupCancelled("Isaac startup cancelled")


def _close_physical_attachment(physical_attachment) -> None:
    """Close the ROS owner before World.stop without masking a runtime failure."""
    primary_error = sys.exc_info()[1]
    try:
        physical_attachment.close()
    except Exception as close_error:
        if primary_error is None:
            raise
        if hasattr(primary_error, "add_note"):
            primary_error.add_note(
                f"Isaac physical attachment cleanup also failed: {close_error}"
            )


if __name__ == "__main__":
    main()
