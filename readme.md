# Differential GPS ROS 2 Driver

This repo is minimal ROS 2 driver for the Quectel LG580P differential GPS module. You can run this either as a standalone
ROS 2 node using the launch file provided or incorporate it into the a node composition as a component;

## Hardware Setup

By default the dgps module is not in "differential gps" mode and will only publish the gps position of Antenna 1. You can enable 
dgps mode by...

The heading is read from the module as the angle from Antenna 1 to Antenna 2.
**Parallel Antenna Mounting**: If you are mounting the antennas parallel with your vehicle make sure antenna 1 is at the back of the vehicle and 
antenna 2 is at the fron of the vehicle and make sure the `angle` parameter is set to 0. 
**Perpendicular Antenna Mounting**: Many applications require the antennas to be mounted perpendicular to the front of the vehicle, if this is your
mounting pattern, ensure that antenna 1 is the **right** antenna and antenna 2 is the **left** and that `angle` is set to -90.

## Published Topics

The driver publishes a few topics:
 - `/dgps/antenna1/fix`: A NavSatFix msg that is the position of antenna 1 as deduced by the module. This will publish firs. This should be the position of the back or right antenna.
 - `/dgps/antenna2/fix`: A NavSatFix msg that is the position of antenna 2 as calculated based on the heading and baseline. This should be the front or left antenna.
 - '/dgps/center/fix`: A NavSatFix msg that is the average position of the two antennas.
 - `/dgps/heading`: A Float32 msg that is the heading of the vehicle in the ENU frame in radians.
 - `/dgps/orientation`: A QuaternionStamped msg this a full attitude estimate of the vehicle.
 - `/dgps/dfix`: A custom DifferentialNavSatFix msg from dgps_msgs that contains the GPS position of Antenna 1 and the heading.

## Subscribed Topics 

This driver only subscribes to the `/rtcm` topic of type `rtcm_msgs/Message` which will pass RTK corrections to the gps module.

## Parameters

Before using this driver you should ensure the parameters in the launch file match your setup:
 - `zone`: Your UTM zone, this will get over written based on the GPS position.
 - `dev`: The device the gps is at usually: `/dev/serial/by-id/usb-1a86_USB_Dual_Serial_5932003145-if00`.
 - `angle`: The rotation of your antenna setup in degrees, where antenna 1 at the back and antenna 2 at the front is 0 degrees.
 - `baseline`: The distance in meters between your two antennas in meters.

## Running
To run the node standalone use:
```
ros2 launch dgps dgps.launch.py zone:=<zone> dev:=<dev> angle:=<angle> baseline:=<baseline>
```
To run this as a component use:
```python
dgps_node = ComposableNode(
    package="dgps",
    plugin="dgps::DGPSNode",
    name="dgps_node",
    parameters=[{
        'dev'=dgps_dev,
        'zone'=utm_zone,
        'angle'=dgps_angle,
        'baseline'=dgps_baseline
    }]
)
```

