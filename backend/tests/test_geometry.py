import math

import pytest

from app.utils.geometry import (
    angle_diff, distance, heading_to, move_toward, path_length, point_segment_projection,
)


def test_distance_pythagorean():
    assert distance((0, 0), (3, 4)) == 5.0
    assert distance((1, 1), (1, 1)) == 0.0


@pytest.mark.parametrize("b,expected", [
    ((0, 10), 0.0),      # north
    ((10, 0), 90.0),     # east
    ((0, -10), 180.0),   # south
    ((-10, 0), 270.0),   # west
    ((10, 10), 45.0),
])
def test_heading_compass(b, expected):
    assert heading_to((0, 0), b) == pytest.approx(expected)


def test_heading_same_point_is_zero():
    assert heading_to((5, 5), (5, 5)) == 0.0


def test_angle_diff_wraps():
    assert angle_diff(10, 350) == pytest.approx(20)
    assert angle_diff(350, 10) == pytest.approx(-20)
    assert angle_diff(180, 0) == pytest.approx(180)


def test_move_toward_partial_step():
    pos, travelled, reached = move_toward((0, 0), (100, 0), speed=10, dt=1)
    assert pos == pytest.approx((10, 0))
    assert travelled == pytest.approx(10)
    assert not reached


def test_move_toward_does_not_overshoot():
    pos, travelled, reached = move_toward((0, 0), (5, 0), speed=10, dt=1)
    assert pos == (5, 0)
    assert travelled == pytest.approx(5)
    assert reached


def test_path_length():
    assert path_length([(0, 0), (3, 4), (3, 10)]) == pytest.approx(11)


def test_point_segment_projection_interior_and_clamped():
    p, t = point_segment_projection((5, 5), (0, 0), (10, 0))
    assert p == pytest.approx((5, 0)) and t == pytest.approx(0.5)
    p, t = point_segment_projection((-5, 5), (0, 0), (10, 0))
    assert p == (0, 0) and t == 0.0
    p, t = point_segment_projection((50, 5), (0, 0), (10, 0))
    assert p == (10, 0) and t == 1.0
