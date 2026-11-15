"""
Test parsing and validation of the canonical Isaac runtime configuration.

Responsibility: Preserve CLI values and reject missing robot or scene assets before startup.
"""

from pathlib import Path

import pytest


from franka_motion_runtime.simulation.isaac.runtime.config import (
    CubeConfig,
    IsaacRunnerConfig,
    StereoCameraConfig,
    parse_runner_config,
    validate_runner_config,
    validate_runner_paths,
)


def test_parse_runner_config_preserves_table_camera_cli_contract():
    config = parse_runner_config(
        [
            '--install_path',
            '/tmp/isaac',
            '--robot-usd',
            '/tmp/franka.usd',
            '--scene-usd',
            '/tmp/scene.usd',
            '--scene-prim',
            '/World/TestFranka',
            '--headless',
            '--fix-base',
            '--add-cube',
            '--cube-size',
            '0.07',
            '--cube-position',
            '0.4',
            '-0.1',
            '0.03',
            '--enable-stereo-camera',
            '--stereo-center-position',
            '1.0',
            '0.0',
            '0.33',
            '--stereo-width',
            '640',
            '--stereo-height',
            '480',
            '--stereo-left-frame-id',
            'left_optical',
            '--stereo-right-frame-id',
            'right_optical',
            '--command-topic',
            '/commands',
            '--state-topic',
            '/states',
            '--time-steps',
            '12',
            '--experience',
            '/tmp/base.kit',
        ]
    )

    assert config.install_path == Path('/tmp/isaac')
    assert config.robot_usd == Path('/tmp/franka.usd')
    assert config.scene_usd == Path('/tmp/scene.usd')
    assert config.scene_prim == '/World/TestFranka'
    assert config.headless is True
    assert config.fix_base is True
    assert config.cube.add_cube is True
    assert config.cube.size == 0.07
    assert config.cube.position == (0.4, -0.1, 0.03)
    assert config.stereo.enabled is True
    assert config.stereo.center_position == (1.0, 0.0, 0.33)
    assert config.stereo.width == 640
    assert config.stereo.height == 480
    assert config.stereo.left_frame_id == 'left_optical'
    assert config.stereo.right_frame_id == 'right_optical'
    assert config.command_topic == '/commands'
    assert config.state_topic == '/states'
    assert config.time_steps == 12
    assert config.experience == '/tmp/base.kit'


def test_validate_runner_paths_accepts_existing_robot_usd_and_scene(tmp_path: Path):
    isaac_root = tmp_path / 'isaac'
    isaac_root.mkdir()
    robot_usd = tmp_path / 'franka.usd'
    scene_usd = tmp_path / 'scene.usd'
    robot_usd.write_text('# usd\n')
    scene_usd.write_text('# usd\n')

    config = parse_runner_config(
        [
            '--install_path',
            str(isaac_root),
            '--robot-usd',
            str(robot_usd),
            '--scene-usd',
            str(scene_usd),
        ]
    )

    validate_runner_paths(config)


def test_validate_runner_paths_rejects_missing_robot_asset(tmp_path: Path):
    isaac_root = tmp_path / 'isaac'
    isaac_root.mkdir()
    config = parse_runner_config(['--install_path', str(isaac_root)])

    with pytest.raises(ValueError, match='--robot-usd'):
        validate_runner_paths(config)


@pytest.mark.parametrize(
    ('field_name', 'config'),
    [
        ('time_steps', IsaacRunnerConfig(time_steps=-1)),
        ('cube.size', IsaacRunnerConfig(cube=CubeConfig(size=-0.1))),
        (
            'stereo.center_position',
            IsaacRunnerConfig(
                stereo=StereoCameraConfig(
                    center_position=(float('nan'), 0.0, 1.0)
                )
            ),
        ),
        ('stereo width', IsaacRunnerConfig(stereo=StereoCameraConfig(width=0))),
        (
            'near_clip',
            IsaacRunnerConfig(
                stereo=StereoCameraConfig(near_clip_m=2.0, far_clip_m=1.0)
            ),
        ),
        ('command_topic', IsaacRunnerConfig(command_topic='')),
        (
            'left_frame_id',
            IsaacRunnerConfig(
                stereo=StereoCameraConfig(left_frame_id='/invalid')
            ),
        ),
        (
            'left_frame_id',
            IsaacRunnerConfig(
                stereo=StereoCameraConfig(left_frame_id='left_optical/')
            ),
        ),
        (
            'left_frame_id',
            IsaacRunnerConfig(
                stereo=StereoCameraConfig(left_frame_id='robot//left_optical')
            ),
        ),
        ('scene_prim', IsaacRunnerConfig(scene_prim='World/Franka')),
    ],
)
def test_runner_validation_rejects_unsafe_values(
    field_name: str,
    config: IsaacRunnerConfig,
):
    with pytest.raises(ValueError, match=field_name):
        validate_runner_config(config)
