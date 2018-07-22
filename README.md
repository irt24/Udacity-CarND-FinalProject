Project Highlights
===
TL;DR: [video](https://photos.app.goo.gl/M9vKVHGetSomF5Rr7)

This is a complete individual submission for the final project of the Udacity Self-Driving Car Nanodegree. It is **not** part of a team submission, and is therefore meant to run on a simulator. The associated e-mail address is turc.raluca@gmail.com.

The code mostly follows the walk-throughs, with some additional handling of the simulator lag. The single hardest aspect of the project was dealing with the high resource demands of the simulator. If not enough resource demands are allocated, the simulator introduces lag, which critically affects driving quality: the position of the car in the video stream does not agree with the reported position of the car, and errors keep accummulating.

Here are the set-ups that I experimented with:
1. **Capstone workspace**. Despite having GPU enabled, the workspace could not keep up with the simulator demands. Regardless of how much the hyperparameters were tuned, the car simply drifted away from the road.
1. **MacBook Air (ROS in UbuntuVM + simulator in MacOS)**. Running ROS in a local Ubuntu VM with port forwarding and the simualtor in MacOS led to similar behavior.
1. **MacBook Pro (ROS in UbuntuVM + simulator in MacOS)**. This was the most reasonable configuration, even though I had to carefully tweak it. Using the same hyperparameters as in the previous two configurations, with camera off, the car follows the waypoints smoothly. However, when turning the camera on, the lag starts to become visible. To keep the computational load under control, I had to turn the camera off periodically, whenever the car was far away from traffic lights. Also, I couldn't record the screen at the same time (using e.g QuickTime), because the car would quickly start drifting away. That explains the potato quality :)

I implemented one additional hack to deal with the simulator lag. Since the true car position lags behind the reported one, the car doesn't stop at traffic lights; when the car in the video stream gets close to a stop line, it mistakenly thinks it has already passed it. So whenever the closest stop line is less than 10 waypoints behind, I make the assumption that the car hasn't passed it. This strategy, combined with carefully turning off the camera at the right time, gets the car to smoothly follow the waypoints and stop at the red traffic lights. See the video linked above.

Submission checklist:
===
- *Launch correctly using the launch files provided in the capstone repro.*
  There were no changes to the launch scripts.
- *Smoothly follow waypoints in the simulator*.
  Given the right computational resources, the car does smootly follow the waypoints. Turning the camera on for long periods of time gets the car off the road. I checked that processing the camera data itself is not the cause of drift by running the simulation with camera on while completely ignoring the traffic light information extracted from the camera and observing the car get off the road.
- *Respect the target top speed for the waypoints.*
  `waypoints_updater` reads in the configured speed limit and, by default, assigns this speed to all waypoints. Later, it might decide to decelerate some of the points in anticipation of red traffic lights. Experimentally, setting `MAX_THROTTLE = 0.2` seems to keep the car within the legal speed limit.
- *Stop at traffic lights when needed.*
  The car does stop at traffic lights after implementing the hack that deals with the lag (see explanation above).
- *Stop and restart PID controllers depending on the state of /vehicle/dbw_enabled.*
  `twist_controller` checks whether `dbw` is enabled; when not, it resets the PID controller and sets the throttle, break and steering angle to 0.
- *Publish throttle, steering, and brake commands at 50Hz.*
  The loop in `dbw_node` publishes the three values at a rate of 50 Hz. 
  
  
