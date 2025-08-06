from numpy import cos, sin, tan, pi, sign
from config import *

def ackermann(wheelbase, x, y, theta, delta, velocity, duration = 1/FPS):
    """
    Bicycle (Ackermann) model with signed curvature and correct screen coords:
      - wheelbase:  distance between axles
      - (x,y,theta): current pose (theta from +x toward forward-axis)
      - delta:       steering angle (rad, signed)
      - velocity:    forward speed (px/sec)
      - duration:    time to integrate (sec, >0)

    Returns (x_new, y_new, theta_new).
    """
    # Straight-line case
    if abs(delta) < 0.1: # smooth robot to straight lines if angle is "small enough"
        x_new     = x + velocity * duration * cos(theta)
        y_new     = y - velocity * duration * sin(theta)
        theta_new = theta
    else:
        # signed curvature
        sgn     = -sign(delta)                   # +1 right, -1 left
        R_mag   = wheelbase / abs(tan(delta))   # positive radius magnitude
        omega   = velocity / R_mag              # positive angular rate
        theta_new = theta + sgn * omega * duration  # new heading

        # signed radius
        R       = sgn * R_mag

        # analytic integration
        x_new = x + R * (sin(theta_new) - sin(theta))
        y_new = y + R * (cos(theta_new) - cos(theta))

    # wrap heading into [-π, π)
    theta_new = (theta_new + pi) % (2*pi) - pi
    return x_new, y_new, theta_new

def ackermann_samples(wheelbase, x0, y0, theta0, delta, velocity, duration):
    """
    Sample the nonrecursive Ackermann model at t = i/fps for i=1..N
    where N = round(total_time * fps). Returns a list of (x,y,theta).
    Ensures that the final sample at t=N/fps == total_time matches the analytic end point.
    """
    dt = 1.0 / FPS
    N  = int(round(duration * FPS))
    poses = []

    if abs(delta) < 0.1:  # smooth robot to straight lines if angle is "small enough"
        # straight‐line sampling
        for i in range(1, N+1):
            t = i * dt
            xi = x0 + velocity * t * cos(theta0)
            yi = y0 - velocity * t * sin(theta0)
            poses.append((xi, yi, theta0))
        return poses

    # turning sampling with signed radius
    sgn   = -sign(delta) # minus added to correct turning direction
    R = wheelbase / abs(tan(delta))
    omega = velocity / R

    for i in range(1, N+1):
        t       = i * dt
        delta = sgn * omega * t                 # signed heading change
        theta_i = theta0 + delta

        xi = x0 + sgn * R * (sin(theta_i) - sin(theta0))
        yi = y0 + sgn * R * (cos(theta_i) - cos(theta0))
        # wrap theta for consistency
        theta_i = (theta_i + pi) % (2*pi) - pi
        poses.append((xi, yi, theta_i))

    return poses

def linear_polar(x,y,theta,velocity,duration = 1/FPS):
    x += velocity*(duration)*cos(theta) 
    y -= velocity*(duration)*sin(theta)
    return x,y

def linear_cart(x,y,v_x,v_y,duration = 1/FPS):
    x += v_x*duration
    y -= v_y*duration
    return x,y

