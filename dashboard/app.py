"""Plotly Dash dashboard for the ArduSoar thermal-soaring simulation.

    python -m dashboard.app          # then open http://127.0.0.1:8050

Left panel: inputs (wind, airspeed, battery, map decay, seed, sensor noise) +
Play / Pause / Reset + a playback-speed slider. Right panel: live 2-D map,
a scrolling altitude trace, and battery / home-reach gauges. A dcc.Interval
steps the Engine; the speed slider sets how many sim-ticks run per interval.
"""

from __future__ import annotations

import threading

from dash import Dash, dcc, html, Input, Output, State, ctx
import plotly.graph_objects as go

from config import DT
from dashboard.engine import Engine, Params
from weather.collector import fetch_weather
from weather.processor import make_prior
from weather import soaringmeteo_prior, openmeteo_prior

# --- server-side simulation state (single local user) -----------------------
SIM = {"engine": Engine(Params()), "history": [], "playing": False, "speed": 10}
SIM["history"].append(SIM["engine"].last)
# The Flask dev server is multi-threaded; serialise all engine access so two
# overlapping interval callbacks never step / rebuild the engine concurrently.
_LOCK = threading.Lock()

BOUNDS = SIM["engine"].p.bounds
ALT_WINDOW = 600.0          # seconds shown in the scrolling altitude plot
TRAIL_MAX = 1500            # decimated trail points on the map
CAND_COLOR = {"unsurveyed": "#185FA5", "lift": "#1D9E75",
              "empty": "#B4B2A9", "abandoned": "#D9B36B"}
MODE_COLOR = {"cruise": "#888780", "probe": "#BA7517", "thermal": "#0F6E56"}


def _restart(seed, wx, wy, airspeed, battery, decay, noise):
    p = Params(seed=int(seed), wind=(float(wx), float(wy)), airspeed=float(airspeed),
               battery_wh=float(battery), map_decay_tau=float(decay),
               sensor_noise=bool(noise))
    SIM["engine"] = Engine(p)
    SIM["history"] = [SIM["engine"].last]


# --- figures ----------------------------------------------------------------
def map_fig():
    eng, hist = SIM["engine"], SIM["history"]
    s = hist[-1]
    fig = go.Figure()
    # thermals (live), colour ~ strength, size ~ strength + merge count
    if s["thermals"]:
        tx, ty, tw, tn = zip(*[(c[0], c[1], c[2], c[4]) for c in s["thermals"]])
        fig.add_trace(go.Scatter(
            x=tx, y=ty, mode="markers", name="thermals",
            marker=dict(size=[10 + 3 * w + 6 * (n - 1) for w, n in zip(tw, tn)],
                        color=tw, colorscale="YlOrRd", cmin=0, cmax=5,
                        symbol="star", line=dict(width=0)),
            hovertemplate="thermal W=%{marker.color:.1f}<extra></extra>"))
    # uploaded map points coloured by survey status
    by = {}
    for cx, cy, st in s["cands"]:
        by.setdefault(st, [[], []])
        by[st][0].append(cx); by[st][1].append(cy)
    for st, (xs, ys) in by.items():
        fig.add_trace(go.Scatter(x=xs, y=ys, mode="markers", name=st,
                      marker=dict(size=11, color=CAND_COLOR[st],
                                  line=dict(width=1, color="white"))))
    # glider trail (decimated) + current position
    step = max(1, len(hist) // TRAIL_MAX)
    fig.add_trace(go.Scatter(x=[h["x"] for h in hist[::step]],
                             y=[h["y"] for h in hist[::step]],
                             mode="lines", name="trail",
                             line=dict(color="#185FA5", width=1)))
    fig.add_trace(go.Scatter(x=[s["x"]], y=[s["y"]], mode="markers", name="glider",
                  marker=dict(size=13, color=MODE_COLOR.get(s["mode"], "#185FA5"),
                              line=dict(width=1.5, color="white"))))
    fig.add_trace(go.Scatter(x=[0], y=[0], mode="markers", name="home",
                  marker=dict(size=13, color="#534AB7", symbol="square")))
    # wind arrow
    wx, wy = eng.p.wind
    fig.add_annotation(x=BOUNDS[1] - 250 + wx * 90, y=BOUNDS[3] - 250 + wy * 90,
                       ax=BOUNDS[1] - 250, ay=BOUNDS[3] - 250, xref="x", yref="y",
                       axref="x", ayref="y", showarrow=True, arrowhead=2,
                       arrowsize=1.4, arrowwidth=1.6, arrowcolor="#444441")
    tag = {"complete": " — HOME ✓", "crashed": " — CRASHED"}.get(
        s["mission"], " — RETURN HOME" if s["return_home"] else "")
    fig.update_layout(
        title=f"endurance — aloft {s['t']/60:.1f} min{tag}",
        xaxis=dict(range=[BOUNDS[0] - 60, BOUNDS[1] + 60], title="x (m)",
                   scaleanchor="y", scaleratio=1),
        yaxis=dict(range=[BOUNDS[2] - 60, BOUNDS[3] + 60], title="y (m)"),
        margin=dict(l=40, r=10, t=40, b=40), showlegend=True, height=520,
        legend=dict(orientation="h", y=-0.12), uirevision="map")
    return fig


def alt_fig():
    hist = SIM["history"]; s = hist[-1]; p = SIM["engine"].p
    t0 = max(0.0, s["t"] - ALT_WINDOW)
    # the window is the last ~ALT_WINDOW/DT entries; scan that tail, not all history
    tail = hist[-(int(ALT_WINDOW / DT) + 2):]
    win = [h for h in tail if h["t"] >= t0]
    fig = go.Figure()
    # shade motor-on spans: ONE rectangle per contiguous run, not per tick
    span0 = None
    for h in win:
        if h["motor"] and span0 is None:
            span0 = h["t"]
        elif not h["motor"] and span0 is not None:
            fig.add_vrect(x0=span0, x1=h["t"], fillcolor="#C75D2C", opacity=0.10, line_width=0)
            span0 = None
    if span0 is not None and win:
        fig.add_vrect(x0=span0, x1=win[-1]["t"], fillcolor="#C75D2C", opacity=0.10, line_width=0)
    fig.add_trace(go.Scatter(x=[h["t"] for h in win], y=[h["h"] for h in win],
                  mode="lines", line=dict(color="#185FA5", width=1.6), name="altitude"))
    fig.add_trace(go.Scatter(x=[s["t"]], y=[s["h"]], mode="markers", name="now",
                  marker=dict(size=10, color=MODE_COLOR.get(s["mode"], "#185FA5"))))
    fig.add_hline(y=p.cloud_base, line=dict(color="#D85A30", dash="dash", width=1))
    fig.add_hline(y=0, line=dict(color="#A32D2D", width=1.2))
    fig.update_layout(title="altitude (scrolling; shaded = motor on)",
                      xaxis=dict(range=[t0, max(s["t"], t0 + 60)], title="time (s)"),
                      yaxis=dict(range=[0, max(p.cloud_base + 80, max(h["h"] for h in win) + 50)],
                                 title="altitude (m)"),
                      margin=dict(l=50, r=10, t=40, b=40), showlegend=False,
                      height=250, uirevision="alt")
    return fig


def gauges_fig():
    s = SIM["history"][-1]
    fig = go.Figure()
    fig.add_trace(go.Indicator(
        mode="gauge+number", value=s["soc"] * 100, title={"text": "battery %"},
        domain={"row": 0, "column": 0},
        gauge={"axis": {"range": [0, 100]}, "bar": {"color": "#0F6E56"}}))
    fig.add_trace(go.Indicator(
        mode="number+delta", value=max(0.0, s["margin"]),
        number={"suffix": " m"}, title={"text": "home-reach fuel"},
        domain={"row": 0, "column": 1}))
    fig.update_layout(grid={"rows": 1, "columns": 2}, height=180,
                      margin=dict(l=20, r=20, t=30, b=10))
    return fig


def status_line():
    s = SIM["history"][-1]
    tag = {"complete": "  ✓ HOME — mission complete", "crashed": "  ✗ CRASHED",
           "return_home": "  ⚠ RETURN-HOME"}.get(s["mission"], "")
    return (f"⏱ aloft {s['time_aloft']/60:.1f} min  |  MODE: {s['mission'].upper()}{tag}"
            f"{'  [MOTOR]' if s['motor'] else ''}  |  h={s['h']:.0f} m  |  "
            f"🔋 battery {s['soc']*100:.0f}%  |  "
            f"home {s['d_home']:.0f} m → costs {s['to_home_wh']:.1f} Wh, spare {s['spare_wh']:.1f} Wh  |  "
            f"thermals worked {s['climbs']}")


# --- layout -----------------------------------------------------------------
app = Dash(__name__)
app.title = "ArduSoar dashboard"
server = app.server          # WSGI entry point for gunicorn / deployment


def _num(id_, label, value, mn, mx, step):
    return html.Div([html.Label(label), dcc.Input(id=id_, type="number", value=value,
                    min=mn, max=mx, step=step, style={"width": "100%"})],
                    style={"marginBottom": "10px"})


controls = html.Div([
    html.H3("ArduSoar"),
    html.Div([
        html.Button("▶ Play", id="play", n_clicks=0),
        html.Button("⏸ Pause", id="pause", n_clicks=0, style={"marginLeft": "6px"}),
        html.Button("⏮ Reset", id="reset", n_clicks=0, style={"marginLeft": "6px"}),
    ], style={"marginBottom": "12px"}),
    html.Label("playback speed (sim-ticks / frame)"),
    dcc.Slider(id="speed", min=1, max=300, step=1, value=40,
               marks={1: "1×", 50: "50", 150: "150", 300: "300 (fast)"}),
    html.Hr(),
    html.Label("wind x (m/s)"),
    dcc.Slider(id="wx", min=-3, max=3, step=0.1, value=0.9, marks={-3: "-3", 0: "0", 3: "3"}),
    html.Label("wind y (m/s)"),
    dcc.Slider(id="wy", min=-3, max=3, step=0.1, value=-0.55, marks={-3: "-3", 0: "0", 3: "3"}),
    _num("airspeed", "airspeed (m/s)", 16.0, 10, 25, 0.5),
    _num("battery", "battery (Wh)", 40.0, 0, 100, 5),
    _num("decay", "map decay τ (s)", 4000.0, 500, 8000, 100),
    _num("seed", "seed", 4, 0, 999, 1),
    dcc.Checklist(id="noise", options=[{"label": " sensor noise", "value": "on"}], value=[]),
    html.Hr(),
    html.Div("changes apply on Reset", style={"fontSize": "12px", "color": "#888"}),
    html.Div(id="ctrl-status", style={"display": "none"}),
    html.Hr(),
    html.Label("🌤 real weather (lat, lon)"),
    html.Div([
        dcc.Input(id="lat", type="number", value=43.47, step=0.01, style={"width": "47%"}),
        dcc.Input(id="lon", type="number", value=-80.54, step=0.01,
                  style={"width": "47%", "marginLeft": "6%"}),
    ], style={"marginBottom": "8px"}),
    dcc.Dropdown(id="wx-source", clearable=False, value="openmeteo",
                 options=[{"label": "Open-Meteo (rad/CAPE)", "value": "openmeteo"},
                          {"label": "Open-Meteo W* (Deardorff)", "value": "openmeteo_wstar"},
                          {"label": "SoaringMeteo (GFS)", "value": "soaringmeteo"}],
                 style={"marginBottom": "8px", "fontSize": "12px"}),
    html.Button("🌤 use weather", id="weather", n_clicks=0),
    html.Div(id="weather-status", style={"fontSize": "11px", "color": "#555",
                                         "marginTop": "8px", "lineHeight": "1.4"}),
], style={"width": "260px", "padding": "16px", "verticalAlign": "top",
          "display": "inline-block", "fontFamily": "sans-serif"})

panel = html.Div([
    html.Div(id="status", style={"fontFamily": "monospace", "fontSize": "14px",
                                 "padding": "8px 0"}),
    dcc.Graph(id="map", config={"displayModeBar": False}),
    html.Div([
        html.Div(dcc.Graph(id="alt"), style={"width": "62%", "display": "inline-block"}),
        html.Div(dcc.Graph(id="gauges"), style={"width": "38%", "display": "inline-block",
                                                "verticalAlign": "top"}),
    ]),
    dcc.Interval(id="tick", interval=150, n_intervals=0),
], style={"width": "calc(100% - 300px)", "display": "inline-block",
          "verticalAlign": "top", "padding": "8px"})

app.layout = html.Div([controls, panel])


# --- callbacks --------------------------------------------------------------
@app.callback(Output("ctrl-status", "children"),
              Input("play", "n_clicks"), Input("pause", "n_clicks"),
              Input("reset", "n_clicks"), Input("speed", "value"),
              State("seed", "value"), State("wx", "value"), State("wy", "value"),
              State("airspeed", "value"), State("battery", "value"),
              State("decay", "value"), State("noise", "value"))
def control(_p, _pa, _r, speed, seed, wx, wy, airspeed, battery, decay, noise):
    SIM["speed"] = int(speed or 1)
    trig = ctx.triggered_id
    if trig == "play":
        SIM["playing"] = True
    elif trig == "pause":
        SIM["playing"] = False
    elif trig == "reset":
        with _LOCK:                                  # rebuild + pause atomically,
            _restart(seed, wx, wy, airspeed, battery, decay, "on" in (noise or []))
            SIM["playing"] = False                   # so no in-flight tick can step on
    return trig or ""


@app.callback(Output("weather-status", "children"),
              Input("weather", "n_clicks"),
              State("lat", "value"), State("lon", "value"),
              State("seed", "value"), State("wx-source", "value"))
def use_weather(n, lat, lon, seed, source):
    """Fetch a real forecast for (lat, lon) from the chosen source, build a prior,
    and restart the flight on it (the glider's map is the forecast — no cheat)."""
    if not n:
        return ""
    try:
        lat, lon = float(lat), float(lon)
        if source == "soaringmeteo":
            prior = soaringmeteo_prior.build_prior(lat, lon, bounds=BOUNDS)
        elif source == "openmeteo_wstar":
            prior = openmeteo_prior.build_prior(lat, lon, bounds=BOUNDS)
        else:
            prior = make_prior(fetch_weather(lat, lon), BOUNDS, seed=1)
        with _LOCK:
            SIM["engine"] = Engine(Params.from_weather(prior, seed=int(seed or 4)))
            SIM["history"] = [SIM["engine"].last]
            SIM["playing"] = False
        return (f"✓ {source} {prior['location'].get('time', '')}: "
                f"{prior['thermal_count']} thermals @ ~{prior['thermal_strength_ms']} m/s, "
                f"base {prior['cloud_base_m']} m, wind {prior['wind']} m/s. Press ▶ Play.")
    except Exception as e:
        return f"✗ weather fetch failed: {e}"


@app.callback(Output("map", "figure"), Output("alt", "figure"),
              Output("gauges", "figure"), Output("status", "children"),
              Input("tick", "n_intervals"))
def tick(_n):
    # Take the lock FIRST, then read engine/history/playing under it, so a Reset
    # (which rebuilds them under the same lock) can never be stepped over by a
    # frame that captured the old engine. Non-blocking: if a frame is already in
    # flight, just re-render the current state.
    if _LOCK.acquire(blocking=False):
        try:
            eng = SIM["engine"]
            if SIM["playing"] and not eng.done:
                for _ in range(SIM["speed"]):
                    SIM["history"].append(eng.step())
                    if eng.done:
                        SIM["playing"] = False
                        break
        finally:
            _LOCK.release()
    return map_fig(), alt_fig(), gauges_fig(), status_line()


if __name__ == "__main__":
    import os
    # host 127.0.0.1 = local only; set HOST=0.0.0.0 to share on your LAN
    app.run(debug=False, host=os.environ.get("HOST", "127.0.0.1"),
            port=int(os.environ.get("PORT", "8050")))
