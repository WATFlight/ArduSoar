# Monte Carlo Simulations

Stress-tests the estimator + guidance the way the parent project's
`monte_carlo/` folder does: run many simulations with randomised conditions and
look at the distribution of outcomes, not just one happy-path run.

## What it varies

**Thermal-centre offset** — the thermal is placed at a random Gaussian offset
from its nominal position (`x_sigma = y_sigma = 70 m` by default). The glider
still cruises toward the same waypoint, so each run asks: *can it notice the
lift, lock onto it, and climb, even when the thermal isn't where it expected?*

## Metrics (steady state = last 100 steps)

- **Core distance error** — `|estimated core − true core|`
- **Estimated radius** `R_th` and **strength** `W_0`
- **Steady-state climb** `h_dot`
- **Time to circling** — first time the mode becomes `THERMAL`
- **Success** — reached `THERMAL` mode *and* steady-state climb > 0.3 m/s

## Run

```bash
python -m monte_carlo.run_monte_carlo            # 30 trials
python -m monte_carlo.run_monte_carlo --n 60     # more trials
```

Prints a summary (success rate, mean altitude gain / climb / core error) and
saves `output/monte_carlo_analysis.png` — five metric histograms plus a
success/fail bar chart.

## Typical result

With the default spread, roughly **75 %** of runs succeed. The failures are the
expected large-offset cases: the thermal lands far enough from the cruise path
that the glider never flies close enough to detect meaningful lift, so it
correctly stays in `CRUISE` and continues to the waypoint rather than chasing a
thermal it can't sense — a realistic outcome, even though it counts as a "fail".
