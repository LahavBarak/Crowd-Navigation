5.7.25 - planning the way local planner will follow global plan:
the goal is to give the local control the best waypoint from the global plan as its goal each time.
initially i thought of taking the furthest point along a radius, or the point closest to the goal, but both have false cases.
seemingly, the best way is to use pure pursuit approach - it will have the same functionality as choosing the best waypoint inside a radius in the intuitive example, however, finding the lookahead distance needs to be done wisely - idea is to calculate the average distance between two points in a plan (do this experimentally in advance) then extrapolate the lookahead distance of the worst-case (i.e. the case in which the best waypoint is nearest the robot), which is a straight line.
this might not prove the "overall best" method, but it's relatively quick to implement and should yield good enough results, the goal is, after all, to narrow down the search space of the local planner.

testing 5 maps with 500 random paths each yielded an average distance of appx 9 pixels.
0-agent testing of the global+local planner scheme *GREATLY* improved the path following of the robot.
# DONE - take video of new behaviour for documentation 

robot motion is still very arch-y, might need to look into cost function hyperparameters

5.7.25 - when agents were incorporated into testing robot collided with them very often.
this is *highly likely* due to the agents being outside of the robot's search radius. 
# DONE (somewhat) needs to increase the search radius (and lookahead) to reflect a typical single-move length.
    found a set of parameters that give a more-or-less optimal result.

15.5.25 - agents are colliding with robot too many times. a lot of false-positive collisions.
however in some sense the robot should know how to handle "asshole" behaviours (goalkeeping, robot-ignoring and even robot-chasing)
# TODO need to improve agent motion model to take robot into account.
# TODO need to improve robot safety to handle malicious agent behaviours

26.5.25 - tested RRT tree sizes. found the collision checks are (as expected) limiting the growth, but not only agent checks - also path checks. however, due to the motion model's implementation - resolution changes did not reduce the number of calculations, so had to revisit the motion model implementation.
it is now split to ackermann which computes start-to-finish positions, and ackermann_samples which yields an array of samples 1/FPS distance apart. the latter is to be used for the simulation only, since it is more computationally heavy. the former is computationally efficient and should be able to reduce computational load by reducing resolution of collision checks.
# TODO need to revisit robot.move() and rewrite it to use ackermann_samples instead of ackermann. the relevant call is the one near the end of the function, but the entire logic flow needs to be revisited.

4.6.25 - tested different sampling methods - BAD IDEA. i want to keep it as unconstrained as possible.
however, something popped up - the cost function needs revisiting. especially the part about nodes in the goal region. maybe instead of using an effectively 0-cost function, i should just add a reward element for being in the goal region and use the same cost function? while we're at it - need to revisit the cost accumulation. do i only accumulate distances? or cost as well? i believe it's the latter, but i don't quite remember. anyway, the idea is not only do i care about straight paths at the end - i also want it during the motion planning. same for proximity.
# TODO revisit cost function. focus on: 1. goal-reached reward instead of separate cost function. 2. cost accumulation between nodes, specifically delta and proximity.