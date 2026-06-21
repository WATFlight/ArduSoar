"""Weather-guided MAVLink companion (the strategic layer).

This is the step-3 differentiator: ArduPilot's ArduSoar handles tactical thermal
capture onboard; this companion decides *where today's thermals are* from the
weather pipeline and flies the aircraft there over MAVLink (GUIDED waypoints),
then hands off to ArduSoar. Developed and tested against ArduPilot SITL.
"""
