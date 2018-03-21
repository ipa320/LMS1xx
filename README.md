LMS1xx [![Build Status](https://travis-ci.org/clearpathrobotics/LMS1xx.svg?branch=master)](https://travis-ci.org/clearpathrobotics/LMS1xx)
======

ROS driver for the SICK LMS1xx and LMS5xx families of laser scanners. Originally from [RCPRG](https://github.com/RCPRG-ros-pkg/RCPRG_laser_drivers).

In the LMS1XX package is only an Echo Filter integrated, but the MRS1000 supports a lot more filters, e.g. particle filter, median filter, as described in Manual MRS1000.

Especially the particle filter is particulary useful in outdoor environments due to filtering dust particles, snow, rain Drops etc.

The PC communicates via Ethernet / ColaA Protocal (ASCII) with the laserscanner. The protocol is defined in the Telegram Listing Ranging.

## Usage
### Particle Filter
to activate/deactivate via launch file add the following line. set to false if not stated otherwise.

```
<arg name="particle_filter" default="true"/>
<param name="particle_filter" value="$(arg particle_filter)"/>
```

### Mean Filter
to activate/deactivate via launch file add the following line. set to false if not stated otherwise. number_scans is set to 2 by default.

```
<arg name="mean_filter" default="false"/>
<arg name="number_scans" default="2"/>
<param name="mean_filter" value="$(arg mean_filter)"/>
<param name="number_scans" value="$(arg number_scans)"/>
``` 
number_scans determines the number of scans which are used for averaging. resulting scan_frequency is divided by the number of scans!
