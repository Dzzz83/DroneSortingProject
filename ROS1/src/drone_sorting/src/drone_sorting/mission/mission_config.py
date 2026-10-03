from geometry_msgs.msg import Point


class MissionConfig:
    """Configuration values for the package delivery mission."""

    PICKUP_POSITION = Point(
        x=18.0,
        y=0.0,
        z=2.0,
    )

    DROPOFF_POSITION = Point(
        x=0.0,
        y=0.0,
        z=2.0,
    )

    ARRIVAL_TOLERANCE = 0.25
