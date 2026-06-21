#!/usr/bin/env python3
"""Plot the ArduSoar SITL demo log: altitude vs time, THERMAL segments shaded.

Usage: plot_soaring.py [soaring_log.csv] [out.png]
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

HERE = os.path.dirname(__file__)
src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "soaring_log.csv")
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "soaring_demo.png")

t, alt, mode = [], [], []
with open(src) as f:
    for row in csv.DictReader(f):
        t.append(float(row["t_s"]))
        alt.append(float(row["rel_alt_m"]))
        mode.append(row["mode"])

fig, ax = plt.subplots(figsize=(11, 5))
ax.plot(t, alt, color="#1f4e79", lw=1.3, label="relative altitude")

# Shade THERMAL (circling) segments.
in_thermal = False
start = 0.0
for i, mo in enumerate(mode):
    if mo == "THERMAL" and not in_thermal:
        in_thermal, start = True, t[i]
    elif mo != "THERMAL" and in_thermal:
        ax.axvspan(start, t[i], color="#e08a1f", alpha=0.25)
        in_thermal = False
if in_thermal:
    ax.axvspan(start, t[-1], color="#e08a1f", alpha=0.25)

ax.axhline(350, ls="--", color="#888", lw=1)
ax.text(t[-1], 352, "SOAR_ALT_MAX", ha="right", va="bottom", color="#888", fontsize=9)
ax.set_xlabel("time since arm (s, wall @ speedup 20)")
ax.set_ylabel("relative altitude (m)")
ax.set_title("ArduSoar in SITL — cruise, detect lift, circle (shaded), climb")
ax.legend(handles=[
    plt.Line2D([], [], color="#1f4e79", lw=1.3, label="relative altitude"),
    Patch(facecolor="#e08a1f", alpha=0.25, label="THERMAL (circling)"),
], loc="lower right")
ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(out, dpi=130)
print(f"wrote {out}")
