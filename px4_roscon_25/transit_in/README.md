# transit_in

`transit_in` is a PX4 ROS 2 Interface Library custom mode. Start the node before selecting **Transit In** from QGroundControl while the vehicle is armed and airborne. PX4 registers the mode dynamically through `px4_ros2_cpp`.

## Parameters

The default parameter file is `config/transit_in.yaml` and is installed with the package.

- `transit_in_alt`: target height above the PX4 home altitude, in metres. Every waypoint uses this same altitude; values above 120 m are clamped.
- `waypoints`: flattened array of latitude/longitude pairs in degrees: `[lat_1, lon_1, lat_2, lon_2, ...]`. Any number of pairs is supported and the order is preserved.
- `waypoint_latitudes` and `waypoint_longitudes`: optional parallel arrays. They are used only when `waypoints` is empty.
- `arrival_radius_m`: horizontal radius used to advance to the next waypoint. Altitude must also be within `altitude_tolerance_m` and vertical speed must be settled.
- `max_horizontal_speed_m_s`: requested horizontal speed limit. Values above the built-in 10 m/s safety cap are clamped.
- `max_vertical_speed_m_s`: maximum vertical correction speed used while holding the transit altitude; it is capped at 3 m/s.
- `max_heading_rate_deg_s`: heading slew-rate limit; it is capped at 180 degrees/s.
- `course_heading_min_speed_m_s`: minimum actual horizontal ground speed before a new course heading is accepted.
- `altitude_tolerance_m`: maximum AMSL altitude error allowed at a waypoint.
- `altitude_settle_speed_m_s`: maximum vertical speed allowed when a waypoint is accepted.
- `data_timeout_s`: maximum wait for fresh required telemetry after activation and maximum age for land-detection data.

The installed YAML leaves the route unset as a safety default; edit `config/transit_in.yaml` and add `waypoints` before running it. The route is completed successfully after the final waypoint is inside the arrival radius, at the requested altitude, and vertically settled. Invalid parameters, missing global/home/local position data, stale land telemetry, a landed vehicle, or a disarmed vehicle cause the mode to report failure instead of sending a route.

## Run in the workshop container

Source the dependency overlay and workspace, then run:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash
source install/setup.bash
# Edit px4_roscon_25/transit_in/config/transit_in.yaml in the source workspace first,
# rebuild if you changed the file, or pass -p waypoints:=[lat,lon,...]
ros2 run transit_in transit_in --ros-args --params-file install/transit_in/share/transit_in/config/transit_in.yaml
```

Select **Transit In** in QGroundControl after the node has registered. The mode uses the PX4 global position and home position topics and sends `GotoGlobalSetpointType` setpoints. Heading is updated from the actual PX4 local NED ground velocity (`atan2(v_east, v_north)`), and the previous heading is retained while nearly stationary.
