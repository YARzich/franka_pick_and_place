"""
Test JSON request builders shared by pose-estimation workers.

Responsibility: Preserve camera matrices, model rules, paths, bbox, masks, and coarse poses.
"""

from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_worker_requests import (  # noqa: E402,I100,I101,I201
    RGBD_MODEL,
    RGB_MODEL,
    build_gigapose_prewarm_request,
    build_gigapose_refiner_prewarm_request,
    build_gigapose_refiner_worker_request,
    build_gigapose_worker_request,
    build_megapose_worker_request,
    camera_k_matrix,
    model_uses_depth,
    select_megapose_model,
)


K_VALUES = [100.0, 0.0, 50.0, 0.0, 110.0, 60.0, 0.0, 0.0, 1.0]


def test_camera_k_matrix_is_nested_float_3x3():
    assert camera_k_matrix(K_VALUES) == [
        [100.0, 0.0, 50.0],
        [0.0, 110.0, 60.0],
        [0.0, 0.0, 1.0],
    ]
    with pytest.raises(ValueError):
        camera_k_matrix([1.0, 2.0])


def test_megapose_model_selection_preserves_depth_mode_rules():
    assert select_megapose_model('gigapose', 'auto', 'off', has_depth=True) == RGB_MODEL
    assert (
        select_megapose_model('megapose', 'custom-model', 'auto', has_depth=True)
        == 'custom-model'
    )
    assert select_megapose_model('megapose', 'auto', 'on', has_depth=False) == RGBD_MODEL
    assert select_megapose_model('megapose', 'auto', 'auto', has_depth=True) == RGBD_MODEL
    assert select_megapose_model('megapose', 'auto', 'off', has_depth=True) == RGB_MODEL
    assert model_uses_depth('megapose-1.0-RGBD')
    assert model_uses_depth('megapose-1.0-RGB-multi-hypothesis-icp')
    assert not model_uses_depth(RGB_MODEL)


def test_build_megapose_worker_request_omits_absent_coarse_estimates():
    request = build_megapose_worker_request(
        request_id=3,
        example_name='franka_cube',
        example_dir=Path('/tmp/example'),
        model_name=RGB_MODEL,
        object_label='cube_5cm',
        bsz_images=64,
        bsz_objects=8,
        renderer_workers=1,
        coarse_estimates=None,
    )

    assert request['id'] == 3
    assert request['example_dir'] == '/tmp/example'
    assert 'coarse_estimates' not in request


def test_build_gigapose_worker_request_serializes_paths_bbox_and_optional_mask():
    request = build_gigapose_worker_request(
        request_id=5,
        image_path=Path('/tmp/image.png'),
        mask_path=None,
        camera_k=K_VALUES,
        bbox=(1, 2, 3, 4),
        repo_dir=Path('/repo'),
        dinov2_repo_dir=None,
        root_dir=Path('/data'),
        dataset_name='franka_cube',
        object_id=1,
        checkpoint_path=Path('/checkpoint.pth'),
        template_count=162,
        max_num_dets_per_forward=4,
        translation_scale=0.001,
        num_hypotheses=5,
    )

    assert request['mask_path'] == ''
    assert request['bbox'] == [1, 2, 3, 4]
    assert request['K'][0] == [100.0, 0.0, 50.0]
    assert request['dinov2_repo_dir'] == ''


def test_build_gigapose_refiner_worker_request_keeps_output_label_and_coarse_payload():
    coarse_estimates = {'poses': [[[1.0]]], 'scores': [0.9]}
    request = build_gigapose_refiner_worker_request(
        request_id=8,
        image_path=Path('/tmp/image.png'),
        camera_k=K_VALUES,
        repo_dir=Path('/repo'),
        root_dir=Path('/data'),
        dataset_name='franka_cube',
        object_id=1,
        output_label='cube_5cm',
        megapose_models_root=Path('/models'),
        bsz_images=512,
        bsz_objects=8,
        n_iterations=5,
        coarse_estimates=coarse_estimates,
    )

    assert request['output_label'] == 'cube_5cm'
    assert request['megapose_models_root'] == '/models'
    assert request['coarse_estimates'] is coarse_estimates


def test_gigapose_prewarm_requests_only_carry_backend_configuration():
    coarse = build_gigapose_prewarm_request(
        request_id=9,
        repo_dir=Path('/repo'),
        dinov2_repo_dir=Path('/dinov2'),
        root_dir=Path('/data'),
        dataset_name='franka_cube',
        object_id=1,
        checkpoint_path=Path('/checkpoint.pth'),
        template_count=162,
        max_num_dets_per_forward=4,
        translation_scale=0.001,
    )
    refiner = build_gigapose_refiner_prewarm_request(
        request_id=10,
        repo_dir=Path('/repo'),
        root_dir=Path('/data'),
        dataset_name='franka_cube',
        object_id=1,
        megapose_models_root=Path('/models'),
        bsz_images=512,
        bsz_objects=8,
        n_iterations=5,
    )

    assert coarse['command'] == 'prewarm'
    assert refiner['command'] == 'prewarm'
    for frame_field in ('image_path', 'mask_path', 'bbox', 'K', 'coarse_estimates'):
        assert frame_field not in coarse
        assert frame_field not in refiner
