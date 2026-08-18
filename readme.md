# Differential GPS ROS 2 Driver

This repo contains ROS 2 drivers for the Septentrio mosaic-G5 P3H dual-antenna GNSS receiver. The driver can be run as a standalone node or composed into a larger node composition.

---

## Septentrio mosaic-G5 P3H

### Developer References

 - [mosaic-G5 Firmware v1.0.1 Reference Guide](https://docs.sparkfun.com/SparkFun_GNSS_mosaic-G5_P3/assets/component_documentation/firmware/v1.0.1/mosaic-G5%20Firmware%20v1.0.1%20Reference%20Guide.pdf) — chip reference guide; see Appendix C for NMEA sentence definitions.
 - [ArduSimple simpleRTK4Heading User Guide](https://www.ardusimple.com/user-guide-simplertk-4-heading/) — RTK heading setup and operation.
 - [ArduSimple mosaic-G5 Setup and Configuration Guide](https://www.ardusimple.com/how-to-configure-septentrio-mosaic-g5/#connect) — initial device configuration walkthrough.
 - [RxTools User Manual](https://www.septentrio.com/system/files/support/rxtools_v26.1.0_user_manual.pdf) — Septentrio's desktop tool for receiver configuration and logging.

### Hardware Setup

The mosaic-G5 P3H exposes two USB CDC serial ports. USB1 (`-if00`) streams NMEA sentences and USB2 (`-if02`) receives RTCM corrections. If you install the udev rule from the `udev/` directory, the ports will be available as `/dev/septentrio_nmea` and `/dev/septentrio_rtcm`.

The heading is reported directly by the receiver as the azimuth from antenna 1 to antenna 2.

**Parallel Antenna Mounting**: Mount antenna 1 at the back of the vehicle and antenna 2 at the front. Set `angle` to 0.

**Perpendicular Antenna Mounting**: Mount antenna 1 on the right side of the vehicle and antenna 2 on the left. Set `angle` to -90.

### Published Topics

 - `/sept/antenna1/fix` — `NavSatFix`: position of antenna 1 as reported by the receiver.
 - `/sept/enu/heading` — `Float64`: vehicle heading in the ENU frame (radians).
 - `/sept/enu/heading_deg` — `Float64`: vehicle heading in the ENU frame (degrees).
 - `/sept/enu/orientation` — `QuaternionStamped`: full attitude in the ENU frame (`base_link`).
 - `/sept/enu/dfix` — `DifferentialNavSatFix`: antenna 1 position plus ENU heading.
 - `/sept/ned/heading` — `Float64`: vehicle heading in the NED frame (radians).
 - `/sept/ned/heading_deg` — `Float64`: vehicle heading in the NED frame (degrees).
 - `/sept/ned/orientation` — `QuaternionStamped`: full attitude in the NED frame.
 - `/sept/ned/dfix` — `DifferentialNavSatFix`: antenna 1 position plus NED heading.

### Subscribed Topics

 - `/rtcm` — `rtcm_msgs/Message`: RTK correction stream forwarded to the receiver over USB2.

### Parameters

 - `zone`: UTM zone (e.g. `18S`).
 - `nmea_dev`: Serial device for the NMEA output stream (USB1), e.g. `/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device_0100008040-if00` or `/dev/septentrio_nmea`.
 - `nmea_baud`: Baud rate for the NMEA port (default `115200`; ignored on USB-CDC).
 - `rtcm_dev`: Serial device for RTCM input (USB2), e.g. `/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device_0100008040-if02` or `/dev/septentrio_rtcm`. Set to empty string to disable RTCM forwarding.
 - `rtcm_baud`: Baud rate for the RTCM port (default `115200`; ignored on USB-CDC).
 - `angle`: Rotation in degrees to align the antenna baseline with the vehicle forward axis.
 - `baseline`: Fallback antenna separation in meters, used when the receiver does not report a baseline vector.

### Running

Standalone node:
```
ros2 launch dgps septentrio.launch.py zone:=<zone> nmea_dev:=<nmea_dev> rtcm_dev:=<rtcm_dev> angle:=<angle> baseline:=<baseline>
```

As a composable component:
```python
septentrio_node = ComposableNode(
    package="dgps",
    plugin="dgps::SeptentrioNode",
    name="septentrio_node",
    parameters=[{
        'nmea_dev': nmea_dev,
        'rtcm_dev': rtcm_dev,
        'utm_zone': utm_zone,
        'angle': dgps_angle,
        'baseline': dgps_baseline
    }]
)
```
