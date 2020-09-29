# OpenSim and ROS example

This is a short working example of how to run OpenSim as a ROS node.

There are three nodes inside the simbody_msm package. The talker, just publishes some random value for a mass in a topic. The listener takes that value and generates a sinusoidal muscle activation profile for the biceps and triceps (looks like [this](https://www.youtube.com/watch?v=ouXK6XrXsyM) )

The third node (fkine) is a more advanced example of running an OpenSim simulation starting with a specific state for the joints (position and velocity) and a muscle activation profile. I haven't tested this one extensively, but it should work.
