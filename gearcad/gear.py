"""Parametric involute spur gear and meshed gear pair.

All dimensions in millimeters, angles in degrees unless noted.
Standard full-depth teeth: addendum = 1.0 m, dedendum = 1.25 m.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cadquery as cq


def inv(alpha_rad: float) -> float:
    """Involute function: inv(a) = tan(a) - a."""
    return math.tan(alpha_rad) - alpha_rad


@dataclass
class GearSpec:
    module: float = 3.0            # m: pitch diameter / teeth
    teeth: int = 20
    pressure_angle: float = 20.0
    face_width: float = 20.0
    bore_d: float = 20.0
    keyway_w: float = 6.0          # 0 for no keyway
    keyway_depth: float = 2.8
    backlash: float = 0.10         # total backlash at the pitch circle, split between gears
    flank_points: int = 14         # points per involute flank

    @property
    def pitch_d(self) -> float:
        return self.module * self.teeth

    @property
    def base_d(self) -> float:
        return self.pitch_d * math.cos(math.radians(self.pressure_angle))

    @property
    def outside_d(self) -> float:
        return self.pitch_d + 2 * self.module

    @property
    def root_d(self) -> float:
        return self.pitch_d - 2.5 * self.module

    @property
    def circular_pitch(self) -> float:
        return math.pi * self.module

    def check(self) -> None:
        # Minimum teeth to avoid undercut with 20 deg full-depth teeth is 17
        min_teeth = math.ceil(2 / math.sin(math.radians(self.pressure_angle)) ** 2)
        if self.teeth < min_teeth:
            raise ValueError(f"{self.teeth} teeth would undercut; use at least {min_teeth}")
        if self.bore_d + 2 * self.keyway_depth >= self.root_d - 2 * self.module:
            raise ValueError("bore and keyway leave too little rim under the teeth")
        if self.module <= 0 or self.face_width <= 0:
            raise ValueError("module and face width must be > 0")


def _tooth_segments(s: GearSpec):
    """Per tooth: (gap point on root, [root->base radial pts], right flank, left flank, [base->root])."""
    z = s.teeth
    rp, rb, ra, rf = s.pitch_d / 2, s.base_d / 2, s.outside_d / 2, s.root_d / 2
    phi = math.radians(s.pressure_angle)
    psi_p = math.pi / (2 * z) - (s.backlash / 2) / (2 * rp)
    r_start = max(rb, rf)
    radii = [r_start + (ra - r_start) * i / (s.flank_points - 1) for i in range(s.flank_points)]

    def half_angle(r: float) -> float:
        return psi_p + inv(phi) - inv(math.acos(min(1.0, rb / r)))

    def pol(r, a):
        return (r * math.cos(a), r * math.sin(a))

    for k in range(z):
        g = 2 * math.pi * k / z
        gap = pol(rf, g - math.pi / z)
        right = [pol(r, g - half_angle(r)) for r in radii]
        left = [pol(r, g + half_angle(r)) for r in reversed(radii)]
        pre = [pol(rf, g - half_angle(r_start))] if rb > rf else []
        post = [pol(rf, g + half_angle(r_start))] if rb > rf else []
        yield gap, pre, right, left, post


def _outline_wire(s: GearSpec) -> cq.Workplane:
    """Gear outline with spline involute flanks and straight root/tip segments."""
    segs = list(_tooth_segments(s))
    wp = cq.Workplane("XY").moveTo(*segs[0][0])
    for i, (gap, pre, right, left, post) in enumerate(segs):
        if i > 0:
            wp = wp.lineTo(*gap)
        for p in pre:
            wp = wp.lineTo(*p)
        wp = wp.lineTo(*right[0]) if not pre else wp
        wp = wp.spline(right[1:], includeCurrent=True)
        wp = wp.lineTo(*left[0])                       # across the tooth tip
        wp = wp.spline(left[1:], includeCurrent=True)
        for p in post:
            wp = wp.lineTo(*p)
    return wp.close()


def _outline_points(s: GearSpec) -> list[tuple[float, float]]:
    z = s.teeth
    rp, rb, ra, rf = s.pitch_d / 2, s.base_d / 2, s.outside_d / 2, s.root_d / 2
    phi = math.radians(s.pressure_angle)
    # Half tooth thickness angle at the pitch circle, minus half of this gear's backlash share
    psi_p = math.pi / (2 * z) - (s.backlash / 2) / (2 * rp)
    r_start = max(rb, rf)
    radii = [r_start + (ra - r_start) * i / (s.flank_points - 1) for i in range(s.flank_points)]

    def half_angle(r: float) -> float:
        alpha = math.acos(min(1.0, rb / r))
        return psi_p + inv(phi) - inv(alpha)

    pts = []
    for k in range(z):
        g = 2 * math.pi * k / z
        gap = g - math.pi / z
        pts.append((rf * math.cos(gap), rf * math.sin(gap)))
        a0 = g - half_angle(r_start)
        if rb > rf:  # radial line from root up to the base circle
            pts.append((rf * math.cos(a0), rf * math.sin(a0)))
        for r in radii:                       # right flank, going out
            a = g - half_angle(r)
            pts.append((r * math.cos(a), r * math.sin(a)))
        for r in reversed(radii):             # left flank, coming back in
            a = g + half_angle(r)
            pts.append((r * math.cos(a), r * math.sin(a)))
        if rb > rf:
            a1 = g + half_angle(r_start)
            pts.append((rf * math.cos(a1), rf * math.sin(a1)))
    return pts


def build_gear(spec: GearSpec | None = None) -> cq.Workplane:
    """Gear centered on the origin, teeth in the XY plane, one tooth on +X."""
    s = spec or GearSpec()
    s.check()
    gear = _outline_wire(s).extrude(s.face_width)
    gear = gear.faces(">Z").workplane().hole(s.bore_d)
    if s.keyway_w > 0:
        key = (cq.Workplane("XY")
               .center(s.bore_d / 2 + s.keyway_depth / 2 - 0.5, 0)
               .rect(s.keyway_depth + 1.0, s.keyway_w).extrude(s.face_width))
        gear = gear.cut(key)
    return gear


def center_distance(g1: GearSpec, g2: GearSpec) -> float:
    if g1.module != g2.module or g1.pressure_angle != g2.pressure_angle:
        raise ValueError("gears must share module and pressure angle to mesh")
    return (g1.pitch_d + g2.pitch_d) / 2


def build_pair(pinion: GearSpec, gear: GearSpec) -> tuple[cq.Workplane, cq.Workplane]:
    """Pinion at the origin, gear at +X, rotated so a pinion tooth sits in a gear gap."""
    a = center_distance(pinion, gear)
    p = build_gear(pinion)
    g = (build_gear(gear)
         .rotate((0, 0, 0), (0, 0, 1), 180 - 180 / gear.teeth)
         .translate((a, 0, 0)))
    return p, g


def contact_ratio(g1: GearSpec, g2: GearSpec) -> float:
    """Transverse contact ratio: average number of teeth in mesh (should be > 1.2)."""
    a = center_distance(g1, g2)
    ra1, rb1 = g1.outside_d / 2, g1.base_d / 2
    ra2, rb2 = g2.outside_d / 2, g2.base_d / 2
    phi = math.radians(g1.pressure_angle)
    path = math.sqrt(ra1**2 - rb1**2) + math.sqrt(ra2**2 - rb2**2) - a * math.sin(phi)
    base_pitch = math.pi * g1.module * math.cos(phi)
    return path / base_pitch
