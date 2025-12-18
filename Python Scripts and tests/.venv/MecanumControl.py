import math
from dataclasses import dataclass

@dataclass
class WheelRotations:
    m1: float  # front-left
    m2: float  # front-right
    m3: float  # rear-left
    m4: float  # rear-right

def normalize_angle(angle: float) -> float:
    """ Normalize angle to range [-pi, pi]. """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle

def mecanum_ik(dx, dy, dtheta, r, L, W) -> WheelRotations:
    """
    Inverse kinematics: compute wheel rotations (rad) for mecanum drive
    from local-frame displacement (dx, dy, dtheta).
    r = wheel radius
    L = half the robot length
    W = half the robot width
    """
    l = L + W
    m1 = ( dx - dy - l * dtheta ) / r  # front-left
    m2 = ( dx + dy + l * dtheta ) / r  # front-right
    m3 = ( dx + dy - l * dtheta ) / r  # rear-left
    m4 = ( dx - dy + l * dtheta ) / r  # rear-right
    return WheelRotations(m1, m2, m3, m4)

def mecanum_to_pose(x_curr, y_curr, theta_curr,
                    x_goal, y_goal, theta_goal,
                    r, L, W) -> WheelRotations:
    """
    Convert global target pose into wheel rotations (rad).
    """
    dx_world = x_goal - x_curr
    dy_world = y_goal - y_curr
    dtheta = normalize_angle(theta_goal - theta_curr)

    # Transform into robot (local) frame
    dx = math.cos(-theta_curr) * dx_world - math.sin(-theta_curr) * dy_world
    dy = math.sin(-theta_curr) * dx_world + math.cos(-theta_curr) * dy_world

    return mecanum_ik(dx, dy, dtheta, r, L, W)
