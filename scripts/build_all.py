"""Build the gear pair and export STEP, STL, SVG drawings, a PNG render and a report.

Run from the repo root:  python scripts/build_all.py
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import cadquery as cq  # noqa: E402
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

from gearcad import GearSpec, build_pair, center_distance, contact_ratio  # noqa: E402

OUT = ROOT / "output"
OUT.mkdir(exist_ok=True)
STEEL = 7850.0  # kg/m^3


def render_png(parts, path, colors, elev=55, azim=-70):
    fig = plt.figure(figsize=(7, 5), dpi=150)
    ax = fig.add_subplot(111, projection="3d")
    light = np.array([0.4, -0.6, 0.7])
    light /= np.linalg.norm(light)
    allpts = []
    for part, base in zip(parts, colors):
        verts, tris = part.val().tessellate(0.15, 0.15)
        pts = np.array([(v.x, v.y, v.z) for v in verts])
        faces = pts[np.array(tris)]
        n = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        shade = 0.35 + 0.65 * np.abs(n @ light)
        ax.add_collection3d(Poly3DCollection(faces, facecolors=np.clip(np.outer(shade, base), 0, 1),
                                             edgecolor="none"))
        allpts.append(pts)
    pts = np.vstack(allpts)
    c = (pts.max(0) + pts.min(0)) / 2
    span = (pts.max(0) - pts.min(0)).max() / 2
    for setter, ci in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), c):
        setter(ci - span, ci + span)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(path, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def main():
    pin, gear = GearSpec(teeth=20, bore_d=20), GearSpec(teeth=40, bore_d=25)
    P, G = build_pair(pin, gear)
    cq.exporters.export(P, str(OUT / "pinion_20T.step"))
    cq.exporters.export(G, str(OUT / "gear_40T.step"))
    cq.exporters.export(P.add(G), str(OUT / "gear_pair_assembly.step"))
    for name, part in (("pinion_20T", P), ("gear_40T", G)):
        cq.exporters.export(part, str(OUT / f"{name}.stl"), tolerance=0.05, angularTolerance=0.15)
    both = P.union(G)
    cq.exporters.export(both, str(OUT / "gear_pair_top.svg"), opt={
        "projectionDir": (0, 0, 1), "showHidden": False, "width": 800, "height": 450, "strokeWidth": 0.3})
    render_png([P, G], OUT / "gear_pair.png", [(0.55, 0.62, 0.70), (0.72, 0.62, 0.45)])

    def info(s, part):
        vol = part.val().Volume()
        return {"teeth": s.teeth, "module_mm": s.module, "pressure_angle_deg": s.pressure_angle,
                "pitch_d_mm": s.pitch_d, "base_d_mm": round(s.base_d, 3), "outside_d_mm": s.outside_d,
                "root_d_mm": s.root_d, "face_width_mm": s.face_width, "bore_d_mm": s.bore_d,
                "volume_mm3": round(vol, 1), "mass_kg_steel": round(vol * 1e-9 * STEEL, 3)}

    report = {"pinion": info(pin, P), "gear": info(gear, G),
              "ratio": gear.teeth / pin.teeth,
              "center_distance_mm": center_distance(pin, gear),
              "contact_ratio": round(contact_ratio(pin, gear), 3),
              "backlash_mm": pin.backlash}
    (OUT / "gear_report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
