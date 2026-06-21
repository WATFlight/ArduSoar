"""Ground-side path planning (our scope).

Per the team split: we own the GROUND path planning — turn today's weather into an
ordered route of thermal waypoints toward a goal, and export it as an uploadable
path. The Pi 5 interprets the uploaded path (+ vision) and the flight controller
flies it; neither is built here.

    weather prior  ->  plan_route()  ->  ordered lat/lon waypoints  ->  .waypoints / .json
"""
