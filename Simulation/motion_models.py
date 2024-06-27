from numpy import cos, sin, tan, pi
from config import *

def ackermann(wheelbase, x, y, theta, delta, velocity, duration = 1/FPS):
    '''
    given state, command and duration - progress dynamic model of vehicle
    returns new state after motion.
    '''
    if delta == 0: # linear motion
        x += velocity*(duration)*cos(theta) 
        y -= velocity*(duration)*sin(theta)
        alpha = 0
    else: 
        if (duration < 1/FPS):
            return x,y,theta
        else:
            x,y,theta = ackermann(wheelbase,x,y,theta,delta,velocity,duration-(1/FPS))
            rotation_radius = wheelbase/tan(delta)
            alpha = (velocity*(1/FPS))/rotation_radius # calculate angle of actual rotation, "1" being a placeholder for 1 time unit, meaning, alpha = v*t/r, where t=1

            x += rotation_radius*(cos(alpha+theta-pi/2) - cos(theta-pi/2))
            y -= rotation_radius*(sin(alpha+theta-pi/2) - sin(theta-pi/2))
            theta += alpha/2 
    return x,y,theta

def linear(x,y,theta,velocity,duration = 1/FPS):
    x += velocity*(duration)*cos(theta) 
    y -= velocity*(duration)*sin(theta)
    return x,y