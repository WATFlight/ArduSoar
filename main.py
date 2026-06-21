"""Entry point: run the simulation and save the plots.

    python main.py            # basic circling (R = 0.5 * R_th)
    python main.py --optimize # grid-search circling radius for max net climb
"""

import argparse

from simulator.simulation import run_simulation
from simulator.plotting import plot_results


def main():
    parser = argparse.ArgumentParser(description="ArduSoar simulation")
    parser.add_argument(
        "--optimize",
        action="store_true",
        help="use grid-search optimised circling radius instead of 0.5 * R_th",
    )
    parser.add_argument(
        "--video",
        action="store_true",
        help="also render a 3D animated GIF of the run to output/",
    )
    args = parser.parse_args()

    log = run_simulation(optimize_circling=args.optimize)

    start_h, end_h = log.h[0], log.h[-1]
    modes = set(log.mode)
    print(f"Simulated {log.t[-1]:.0f} s, {len(log.t)} steps.")
    print(f"Altitude: {start_h:.1f} m -> {end_h:.1f} m  (net {end_h - start_h:+.1f} m)")
    print(f"Modes visited: {', '.join(sorted(modes))}")

    out = plot_results(log)
    print(f"Saved plots to {out}")

    if args.video:
        from simulator.render_3d import render_3d

        gif = render_3d(log)
        print(f"Saved 3D animation to {gif}")


if __name__ == "__main__":
    main()
