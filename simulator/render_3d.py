"""3D animation of a soaring run.

Renders the glider's flight path in 3D (x, y, altitude) as an animated GIF: the
glider cruises in, locks onto the thermal, and spirals up its translucent
column. Saved to ``output/`` (no ffmpeg needed — uses Pillow).
"""

import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d projection)

from config import THERMAL_X, THERMAL_Y, THERMAL_R
from simulator.simulation import run_simulation, SimLog

_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "output")
_MODE_COLOR = {"cruise": "#888780", "probe": "#BA7517", "thermal": "#0F6E56"}


def render_3d(log: SimLog = None, filename: str = "soaring_3d.gif", stride: int = 20, fps: int = 20) -> str:
    if log is None:
        log = run_simulation()

    x = np.array(log.x)
    y = np.array(log.y)
    h = np.array(log.h)
    modes = log.mode

    # Subsample so the GIF stays light.
    idx = list(range(0, len(x), stride))
    if idx[-1] != len(x) - 1:
        idx.append(len(x) - 1)

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    xlo = min(x.min(), THERMAL_X - THERMAL_R) - 20
    xhi = max(x.max(), THERMAL_X + THERMAL_R) + 20
    ylo = min(y.min(), THERMAL_Y - THERMAL_R) - 20
    yhi = max(y.max(), THERMAL_Y + THERMAL_R) + 20
    zlo, zhi = h.min() - 20, h.max() + 20
    ax.set_xlim(xlo, xhi)
    ax.set_ylim(ylo, yhi)
    ax.set_zlim(zlo, zhi)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("altitude (m)")
    ax.set_title("ArduSoar — 3D soaring")

    # Static translucent thermal column.
    theta = np.linspace(0, 2 * np.pi, 48)
    zcol = np.linspace(zlo, zhi, 2)
    Th, Zc = np.meshgrid(theta, zcol)
    Xc = THERMAL_X + THERMAL_R * np.cos(Th)
    Yc = THERMAL_Y + THERMAL_R * np.sin(Th)
    ax.plot_surface(Xc, Yc, Zc, color="#D85A30", alpha=0.10, linewidth=0)
    ax.scatter([THERMAL_X], [THERMAL_Y], [zlo], color="#D85A30", marker="*", s=120)

    trail, = ax.plot([], [], [], color="#185FA5", lw=1.3)
    glider, = ax.plot([], [], [], marker="o", color="#D85A30", ms=7)

    def update(frame_i):
        i = idx[frame_i]
        trail.set_data(x[: i + 1], y[: i + 1])
        trail.set_3d_properties(h[: i + 1])
        glider.set_data([x[i]], [y[i]])
        glider.set_3d_properties([h[i]])
        glider.set_color(_MODE_COLOR.get(modes[i], "#D85A30"))
        ax.view_init(elev=24, azim=-60 + 0.4 * frame_i)  # slow orbit
        return trail, glider

    anim = FuncAnimation(fig, update, frames=len(idx), blit=False)

    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    out_path = os.path.abspath(os.path.join(_OUTPUT_DIR, filename))
    anim.save(out_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out_path
