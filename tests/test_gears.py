import math

import pytest

from gearcad import GearSpec, build_gear, build_pair, center_distance, contact_ratio, inv

PIN, GEAR = GearSpec(teeth=20), GearSpec(teeth=40, bore_d=25)


def overlap(a, b) -> float:
    try:
        return a.intersect(b).val().Volume()
    except Exception:  # empty intersection
        return 0.0


@pytest.fixture(scope="module")
def pair():
    return build_pair(PIN, GEAR)


def test_standard_gear_dimensions():
    assert PIN.pitch_d == 60
    assert PIN.outside_d == 66
    assert PIN.root_d == 52.5
    assert PIN.base_d == pytest.approx(60 * math.cos(math.radians(20)))


def test_involute_function():
    assert inv(math.radians(20)) == pytest.approx(0.014904, rel=1e-4)


def test_gear_is_valid_and_right_size():
    g = build_gear(PIN)
    assert g.val().isValid()
    bb = g.val().BoundingBox()
    assert bb.xlen == pytest.approx(PIN.outside_d, abs=0.2)
    assert bb.zlen == pytest.approx(PIN.face_width, abs=0.01)


def test_center_distance_and_ratio():
    assert center_distance(PIN, GEAR) == 90
    assert GEAR.teeth / PIN.teeth == 2


def test_contact_ratio_is_healthy():
    # Spur gears should have a contact ratio above about 1.2
    assert 1.4 < contact_ratio(PIN, GEAR) < 1.9


def test_meshed_pair_does_not_interfere(pair):
    p, g = pair
    assert overlap(p, g) < 1e-3


@pytest.mark.parametrize("deg", [4, 8])
def test_conjugate_rotation_stays_clear(pair, deg):
    p, g = pair
    a = center_distance(PIN, GEAR)
    p2 = p.rotate((0, 0, 0), (0, 0, 1), deg)
    g2 = g.rotate((a, 0, 0), (a, 0, 1), -deg * PIN.teeth / GEAR.teeth)
    assert overlap(p2, g2) < 1e-3


def test_wrong_rotation_collides(pair):
    # Turning only the pinion jams a tooth into the gear: proves the teeth really touch
    p, g = pair
    assert overlap(p.rotate((0, 0, 0), (0, 0, 1), 6), g) > 100


def test_undercut_rejected():
    with pytest.raises(ValueError):
        build_gear(GearSpec(teeth=12))


def test_mismatched_module_rejected():
    with pytest.raises(ValueError):
        center_distance(GearSpec(module=2), GearSpec(module=3))
