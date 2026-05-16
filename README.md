# Sensing and Navigation

GPS, IMU, and dead-reckoning experiments from **EECE 5554: Robot Sensing and Navigation** at Northeastern University. Four labs that walk from raw NMEA bytes off a USB GPS puck all the way to fusing accelerometer, gyro, and magnetometer to reconstruct a car's trajectory through Boston.

The whole point of this course is to stop treating "the sensor" as a black box. Every lab is a noise-vs-signal fight: figure out what the sensor is actually telling you, then figure out what to do about it.

## Why I kept this repo

Most of my robotics work since then (VI-SLAM, Kalman-filter trackers, the surgical robot stack I work on at Globus) assumes you already understand WHY raw IMU integration drifts, WHY GPS scatters around a stationary point, and WHY magnetometers need hard-iron and soft-iron calibration before you can trust a heading. This repo is where I built that intuition the long way: collect data, plot it, get surprised, fix it.

## What's in here

| Lab | Folder | What it does |
|---|---|---|
| 1 | `GPS/` | ROS driver for a USB GPS puck. Parses `$GPGGA` NMEA, converts lat/lon to UTM, publishes a custom `gps` message. Stationary vs walking analysis at Clemente Field (Boston). |
| 2 | `GPS-RTK/` | Same driver pattern, upgraded to a GNSS-RTK puck (`$GNGGA`, multi-constellation, fix-quality flag). Stationary and moving runs, obstructed vs unobstructed, to see how much RTK actually buys you. |
| 3 | `IMU/` | ROS driver for the **VectorNav VN-100**. Parses `$VNYMR` strings, builds `sensor_msgs/Imu` + `MagneticField`, converts Euler to quaternion. 15-minute and 5-hour stationary captures for noise characterisation and Allan deviation. |
| 4 | `Dead Reckoning/` | The payoff lab. Drove Northeastern's NUANCE car around Ruggles Circle, fused IMU + magnetometer + GPS to reconstruct yaw and forward velocity, then a 2D trajectory. |

## Hardware and data

- **GPS puck** — USB NMEA-0183, 4800 baud, `$GPGGA` sentences (Lab 1)
- **GNSS-RTK puck** — `$GNGGA` sentences with quality indicator (Lab 2)
- **VectorNav VN-100** — 9-DOF AHRS, USB serial at 115200 baud, `$VNYMR` ASCII output (Labs 3 and 4)
- **Northeastern NUANCE autonomous car** — driving platform for the dead-reckoning run (Lab 4)

Every lab folder ships its raw rosbags so you can re-run the analysis without owning the hardware:

- `GPS/src/data/{stationary,moving}.bag` and matching `.csv`
- `GPS-RTK/src/gps_driver/data/{Stationary,Move,Unobstructed_Stationary,Unobstructed_Move}.bag`
- `Dead Reckoning/src/analysis/driving_data_nuance.bag` (30 MB, full Ruggles drive)

## Algorithms touched

- **NMEA parsing** for `$GPGGA` and `$GNGGA`, including N/S/E/W sign convention and `ddmm.mmmm` to decimal degrees
- **WGS-84 to UTM** via the `utm` Python package, with zone and band tracked
- **Allan variance / Allan deviation** on a 5-hour stationary IMU log to read off angle random walk and bias instability (MATLAB live script in `IMU/src/analysis/Lab3AllanDeviation.mlx`)
- **Magnetometer calibration** — hard-iron offset and soft-iron ellipse-to-circle scaling, with bank and elevation tilt compensation as a pre-step. Around Ruggles the major-axis scale factor came out to **0.69**.
- **Yaw fusion** — four estimators compared on the same drive: raw IMU yaw, gyro-integrated yaw, magnetometer yaw, and a **complementary filter** combining mag (low-frequency truth) with gyro (high-frequency truth)
- **Forward velocity from accelerometer**, with manual bias re-zeroing at known stationary intervals, compared against GPS-derived velocity (raw integration drifted to ~100 m/s of false negative velocity, so this step matters)

## Sample results

Pulled from the lab reports in this repo:

- IMU stationary noise on the VN-100, accel axes (m/s^2): mean ≈ (1.65, 2.41, -9.37), std ≈ (0.013, 0.013, 0.020). Gyro was the cleanest of the three sensors.
- GPS stationary scatter at Clemente Field: std_easting = 0.117 m, std_northing = 0.479 m, std_alt = 0.134 m over 600 samples. Best-fit-line max error 0.84 m.
- Magnetometer pre-calibration: ellipse centered well off the origin with significant hard-iron offset. Post-calibration: circle on origin. Yaw from corrected mag tracks IMU yaw closely; raw-mag yaw is unusable.

Full plots, tables, and discussion are in the per-lab PDF reports:

- `GPS/src/Report.pdf`
- `GPS-RTK/src/gps_driver/analysis/Report_Lab2.pdf`
- `IMU/src/analysis/Report.pdf`
- `Dead Reckoning/src/analysis/Report_Lab4.pdf`

## Quickstart

These are ROS 1 (Noetic / Melodic) catkin packages. Each lab folder is a standalone package. The package name inside `package.xml` is `lab1` for both the GPS and the GPS-RTK folder; if you want to build them side by side rename one.

```bash
# In a catkin workspace
cd ~/catkin_ws/src
ln -s /path/to/Sensing-and-Navigation/GPS ./lab1_gps
ln -s /path/to/Sensing-and-Navigation/IMU ./lab3_imu
cd ~/catkin_ws && catkin_make
source devel/setup.bash

# Run a driver against a live sensor
rosrun lab1 driver.py _port:=/dev/ttyUSB0 _baudrate:=4800   # GPS
rosrun lab3 imu_driver.py _port:=/dev/ttyUSB0 _baudrate:=115200  # IMU

# Or replay one of the bags
rosbag play "Dead Reckoning/src/analysis/driving_data_nuance.bag"
```

For the analysis, open the `.mlx` MATLAB live scripts in each `analysis/` folder.

## Tech

- **Languages:** Python 2/3 (ROS nodes), MATLAB (analysis)
- **Frameworks:** ROS 1, catkin, `sensor_msgs`, `tf.transformations`
- **Python deps:** `pyserial`, `numpy`, `utm`, `rospy`
- **Course:** EECE 5554, Northeastern University, Khoury / MIE

## Repo layout

```
.
|-- GPS/                # Lab 1: USB GPS puck driver + stationary/walking analysis
|-- GPS-RTK/            # Lab 2: GNSS-RTK driver + obstructed/unobstructed runs
|-- IMU/                # Lab 3: VN-100 driver + Allan deviation
|-- Dead Reckoning/     # Lab 4: IMU + mag + GPS dead-reckoning of NUANCE car
|-- README.md
|-- LICENSE
`-- .gitignore
```

## License

MIT. See [LICENSE](LICENSE).
