"""Unit tests for One-Euro Filter & Quaternion SLERP."""
import numpy as np
import pytest

from tuinadex_teleop.core.filters import OneEuroFilter, slerp


def test_one_euro_filter_smoothing():
    f = OneEuroFilter(n_dim=3, freq_hz=30.0, min_cutoff=1.0, beta=0.01)

    # Constant signal with zero-mean noise
    np.random.seed(42)
    clean_val = np.array([0.1, 0.2, 0.3])
    raw_samples = [clean_val + np.random.normal(0, 0.05, 3) for _ in range(50)]

    filtered = []
    t = 0.0
    for sample in raw_samples:
        filtered.append(f.filter(sample, timestamp=t))
        t += 1.0 / 30.0

    filtered = np.array(filtered)
    # Variance of filtered samples should be substantially lower than raw samples
    raw_var = np.var(raw_samples[10:], axis=0)
    filt_var = np.var(filtered[10:], axis=0)
    assert np.all(filt_var < raw_var)


def test_one_euro_filter_reset():
    f = OneEuroFilter(n_dim=1, freq_hz=30.0)
    out1 = f.filter(np.array([10.0]), timestamp=0.0)
    assert np.isclose(out1[0], 10.0)

    f.reset()
    out2 = f.filter(np.array([0.0]), timestamp=1.0)
    assert np.isclose(out2[0], 0.0)


def test_quaternion_slerp():
    # q0: identity quaternion [1, 0, 0, 0]
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    # q1: 90 deg rotation around Z [cos(pi/4), 0, 0, sin(pi/4)]
    q1 = np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)])

    # t = 0 -> q0
    np.testing.assert_allclose(slerp(q0, q1, 0.0), q0, atol=1e-5)
    # t = 1 -> q1
    np.testing.assert_allclose(slerp(q0, q1, 1.0), q1, atol=1e-5)

    # t = 0.5 -> 45 deg rotation around Z
    expected_mid = np.array([np.cos(np.pi / 8), 0.0, 0.0, np.sin(np.pi / 8)])
    np.testing.assert_allclose(slerp(q0, q1, 0.5), expected_mid, atol=1e-5)

