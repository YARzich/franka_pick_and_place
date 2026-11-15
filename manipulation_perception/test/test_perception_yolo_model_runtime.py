"""
Test YOLO model resolution and optional SAHI runtime construction.

Responsibility: Verify injected model sources, device/class options, and active models.
"""

from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.yolo.yolo_model_runtime import (  # noqa: E402,I100,I101,I201
    active_yolo_model,
    active_yolo_model_path,
    load_yolo_runtime,
    resolve_class_filter,
    resolve_model_source,
    resolve_sahi_device,
)


class _FakeLogger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warn(self, message: str) -> None:
        self.warnings.append(message)


class _FakeYolo:
    names = {0: 'cube', 1: 'mug'}

    def __init__(self, source: str) -> None:
        self.source = source
        self.task = 'segment' if 'mask' in source else 'detect'


class _FakeYoloWorld(_FakeYolo):
    def set_classes(self, classes: list[str]) -> None:
        self.classes = classes


class _FakeAutoDetectionModel:
    calls: list[dict] = []

    @classmethod
    def from_pretrained(cls, **kwargs):
        cls.calls.append(dict(kwargs))
        return {'sahi': kwargs}


def _fake_ultralytics_loader():
    return _FakeYolo, _FakeYoloWorld


def _fake_sahi_loader():
    return _FakeAutoDetectionModel, object()


def test_resolve_model_source_requires_an_existing_local_file(tmp_path):
    model_file = tmp_path / 'cube.pt'
    model_file.write_text('model')

    assert resolve_model_source(str(model_file)) == str(model_file)

    with pytest.raises(FileNotFoundError):
        resolve_model_source(str(tmp_path / 'missing.pt'))
    with pytest.raises(FileNotFoundError):
        resolve_model_source('yolov8s-worldv2.pt')
    with pytest.raises(ValueError, match='model_path'):
        resolve_model_source('')


def test_sahi_device_and_class_filter_resolution():
    assert resolve_sahi_device('') is None
    assert resolve_sahi_device('0') == 'cuda:0'
    assert resolve_sahi_device('cuda:1') == 'cuda:1'
    assert resolve_sahi_device('cpu') == 'cpu'

    assert resolve_class_filter({0: 'cube', 1: 'mug'}, ['cube', '1', 'cube']) == [0, 1]
    assert resolve_class_filter(['cube', 'mug'], ['mug']) == [1]
    assert resolve_class_filter({0: 'cube'}, ['unknown']) is None
    assert resolve_class_filter({0: 'cube'}, []) is None


def test_load_yolo_runtime_uses_mask_model_and_sahi_without_real_dependencies(tmp_path):
    base_model = tmp_path / 'base.pt'
    mask_model = tmp_path / 'mask.pt'
    base_model.write_text('base')
    mask_model.write_text('mask')
    logger = _FakeLogger()
    _FakeAutoDetectionModel.calls = []

    runtime = load_yolo_runtime(
        model_path=str(base_model),
        mask_model_path=str(mask_model),
        use_yolo_world=False,
        open_vocab_classes=[],
        enable_runtime_controls=True,
        confidence=0.35,
        device='0',
        imgsz=640,
        logger=logger,
        ultralytics_loader=_fake_ultralytics_loader,
        sahi_loader=_fake_sahi_loader,
    )

    assert runtime.model.source == str(base_model)
    assert runtime.mask_model.source == str(mask_model)
    assert runtime.mask_model_available is True
    assert runtime.sahi_available is True
    assert runtime.sahi_model == {'sahi': _FakeAutoDetectionModel.calls[0]}
    assert _FakeAutoDetectionModel.calls[0]['device'] == 'cuda:0'
    assert _FakeAutoDetectionModel.calls[0]['image_size'] == 640
    assert logger.warnings == []


def test_active_model_helpers_select_mask_only_when_available():
    model = object()
    mask_model = object()

    assert active_yolo_model(model, mask_model, True) is mask_model
    assert active_yolo_model(model, None, True) is model
    assert active_yolo_model_path('base.pt', 'mask.pt', mask_model, True) == 'mask.pt'
    assert active_yolo_model_path('base.pt', 'mask.pt', None, True) == 'base.pt'
