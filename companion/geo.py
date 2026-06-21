"""Local ENU metres <-> lat/lon, as an equirectangular offset around an origin.

The weather pipeline emits thermal candidates as local metres (x=east, y=north)
relative to a reference lat/lon. MAVLink GUIDED waypoints need global lat/lon.
For the few-km ranges a glider covers, a flat-earth offset is plenty accurate.
"""
import math

_R_EARTH = 6378137.0  # WGS84 equatorial radius, m


def enu_to_latlon(origin_lat, origin_lon, east_m, north_m):
    """(east, north) metres relative to origin -> (lat, lon) degrees."""
    dlat = math.degrees(north_m / _R_EARTH)
    dlon = math.degrees(east_m / (_R_EARTH * math.cos(math.radians(origin_lat))))
    return origin_lat + dlat, origin_lon + dlon


def latlon_to_enu(origin_lat, origin_lon, lat, lon):
    """(lat, lon) degrees -> (east, north) metres relative to origin."""
    north = math.radians(lat - origin_lat) * _R_EARTH
    east = math.radians(lon - origin_lon) * _R_EARTH * math.cos(math.radians(origin_lat))
    return east, north


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * _R_EARTH * math.asin(math.sqrt(a))
