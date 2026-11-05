import math
from mavsdk_grpc.mission import MissionItem


def get_destination_coordinate(lat1_deg: float, lon1_deg: float, distance_m: float, bearing_deg: float) -> tuple:
    """
    Calculates the destination coordinates given a starting coordinate, 
    distance, and bearing.
    
    :param lat1_deg: Starting latitude in degrees
    :param lon1_deg: Starting longitude in degrees
    :param distance_m: Distance to travel in meters
    :param bearing_deg: Bearing in degrees (0 = North, 90 = East, 180 = South, 270 = West)
    :return: Tuple containing (target_latitude, target_longitude) in degrees
    """
    # Earth's mean radius in meters (matches your C++ code)
    R = 6371000.0

    # 1. Convert all inputs from degrees to radians
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    bearing = math.radians(bearing_deg)

    # 2. Calculate the angular distance
    angular_dist = distance_m / R

    # 3. Calculate destination latitude
    lat2 = math.asin(
        math.sin(lat1) * math.cos(angular_dist) +
        math.cos(lat1) * math.sin(angular_dist) * math.cos(bearing)
    )

    # 4. Calculate destination longitude
    y = math.sin(bearing) * math.sin(angular_dist) * math.cos(lat1)
    x = math.cos(angular_dist) - math.sin(lat1) * math.sin(lat2)
    lon2 = lon1 + math.atan2(y, x)

    # 5. Convert results back to degrees
    lat2_deg = math.degrees(lat2)
    lon2_deg = math.degrees(lon2)

    # 6. Normalize longitude to be between -180 and +180 degrees
    # This handles edge cases where the path crosses the International Date Line
    lon2_deg = (lon2_deg + 540) % 360 - 180

    return lat2_deg, lon2_deg


def make_wp(lat: float, lon: float, rel_alt_m: float, speed_m_s: float, vehicle_action: MissionItem.VehicleAction, fly_through: bool = True, acc_radius_m: float = 1.0) -> MissionItem:
    """Helper to construct a MissionItem compatible across MAVSDK v1.x and v2+/v3+."""
    return MissionItem(
        latitude_deg=lat,
        longitude_deg=lon,
        relative_altitude_m=rel_alt_m,
        speed_m_s=speed_m_s,
        is_fly_through=fly_through,
        gimbal_pitch_deg=float('nan'),
        gimbal_yaw_deg=float('nan'),
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=float('nan'),
        camera_photo_interval_s=float('nan'),
        acceptance_radius_m=acc_radius_m,
        yaw_deg=float('nan'),
        camera_photo_distance_m=float('nan'),
        vehicle_action=vehicle_action
    )