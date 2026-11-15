"""
Test exact header identity used by perception synchronization.

Responsibility: Require both a valid timestamp and matching frame_id for related sensor data.
"""

from pathlib import Path
import sys
from types import SimpleNamespace

from std_msgs.msg import Header


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.common.header_sync import HeaderKey, header_key, headers_match  # noqa: E402,I100,I101,I201


def _header(sec=1, nanosec=2, frame_id='camera'):
    header = Header()
    header.stamp.sec = sec
    header.stamp.nanosec = nanosec
    header.frame_id = frame_id
    return header


def test_header_key_requires_stamp_and_frame_id():
    assert header_key(_header()) == HeaderKey(1_000_000_002, 'camera')
    assert header_key(_header(sec=0, nanosec=0)) is None
    assert header_key(_header(sec=-1, nanosec=999_999_999)) is None
    assert header_key(_header(frame_id='')) is None
    assert header_key(None) is None
    assert header_key(SimpleNamespace(frame_id='camera')) is None
    assert header_key(_header(sec=1, nanosec=1_000_000_000)) is None
    assert header_key(_header(sec=-1, nanosec=1_000_000_001)) is None
    for frame_id in (' camera', 'camera ', '/camera', 'camera/', 'camera//optical'):
        assert header_key(_header(frame_id=frame_id)) is None


def test_headers_match_requires_exact_stamp_and_frame_id():
    assert headers_match(_header(sec=3, nanosec=4, frame_id='camera'), _header(sec=3, nanosec=4, frame_id='camera'))
    assert not headers_match(_header(sec=3, nanosec=4), _header(sec=3, nanosec=5))
    assert not headers_match(_header(frame_id='camera'), _header(frame_id='other'))
    assert not headers_match(_header(frame_id=''), _header(frame_id=''))
    assert not headers_match(_header(frame_id='camera'), _header(frame_id=' camera '))
    assert not headers_match(_header(sec=0, nanosec=0), _header(sec=0, nanosec=0))
    assert not headers_match(_header(sec=-1, nanosec=999_999_999), _header(sec=-1, nanosec=999_999_999))
