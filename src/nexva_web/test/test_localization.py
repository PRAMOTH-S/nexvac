"""localization(): honest about AMCL vs SLAM, and about stale transforms."""

import os
import sys
import threading
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import _stubs                                                  # noqa: E402

_stubs.install_aiohttp_stub()

import rclpy.time                                              # noqa: E402
from nexva_web import nav_client                               # noqa: E402

NOW = 100.0


def fake(stamp_s, amcl_pubs, pose=None, cov=0.01, have_tf=True):
    def lookup(*a):
        if not have_tf:
            raise RuntimeError('no transform')
        return types.SimpleNamespace(header=types.SimpleNamespace(
            stamp=rclpy.time.Time(seconds=int(stamp_s),
                                  nanoseconds=int((stamp_s % 1) * 1e9)).to_msg()))
    return types.SimpleNamespace(
        tf_buffer=types.SimpleNamespace(lookup_transform=lookup),
        get_clock=lambda: types.SimpleNamespace(
            now=lambda: rclpy.time.Time(seconds=int(NOW))),
        count_publishers=lambda t: amcl_pubs,
        _lock=threading.Lock(), _pose=pose, _cov_xx=cov, _cov_yy=cov,
        _tf_fresh=None)


def loc(f):
    f._tf_fresh = lambda: nav_client.NavClient._tf_fresh(f)
    return nav_client.NavClient.localization(f)


def test_slam_fresh_tf_without_amcl_is_localized_slam():
    assert loc(fake(NOW - 0.2, 0)) == (True, 'slam')


def test_stale_tf_is_not_localized_even_though_buffer_has_it():
    assert loc(fake(NOW - 30, 0)) == (False, None)
    assert loc(fake(NOW - 30, 1)) == (False, None)


def test_static_tf_stamp_zero_counts_as_live():
    assert loc(fake(0.0, 0)) == (True, 'slam')


def test_no_tf_is_not_localized():
    assert loc(fake(NOW, 0, have_tf=False)) == (False, None)


def test_amcl_uses_covariance():
    assert loc(fake(NOW, 1, pose=object(), cov=0.01)) == (True, 'amcl')
    assert loc(fake(NOW, 1, pose=object(), cov=9.0)) == (False, 'amcl')
    assert loc(fake(NOW, 1, pose=None)) == (True, 'amcl')   # cold start
