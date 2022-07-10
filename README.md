# Odometry
Niri's Odometry is stored here!

## Basic Theory

The **Intel Realsense D435I** camera has an internal inertial measurment unit (IMU). This IMU is capable of providing gyroscopic and acceleration data.
The process of converting from inertial measurements to yaw, pitch, and roll is as follows:

1. Get the data in vector form
2. Figure out the change in angles, using the dt (delta time)
3. Continuously keep adding the changes in angles, analgous to integration but in discrete time
4. Apply a GH-filter as gyroscopic data is highly receptive, but unreliable while acceleration data is long-term stable, but less receptive.

TODO(Not Implemented as of now):
The next step is to double integrate the acceleration data, so as to be able to get real world coordinates.
