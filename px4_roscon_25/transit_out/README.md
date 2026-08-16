# transit_out

`transit_out` is an independent PX4 ROS 2 Interface Library custom mode with the same behavior and parameters as `transit_in`, but registered as **Transit Out**. Start it before selecting **Transit Out** from QGroundControl while the vehicle is armed and airborne.

## Parameters

The default parameter file is `config/transit_out.yaml` and is installed with the package.

- `transit_out_alt`: target height above the PX4 home altitude, in metres. Every waypoint uses this same altitude; values above 120 m are clamped.
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

The installed YAML leaves the route unset as a safety default; edit `config/transit_out.yaml` and add `waypoints` before running it. The route is completed successfully after the final waypoint is inside the arrival radius, at the requested altitude, and vertically settled. Invalid parameters, missing global/home/local position data, stale land telemetry, a landed vehicle, or a disarmed vehicle cause the mode to report failure instead of sending a route.

## Run in the workshop container

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash
source install/setup.bash
# Edit px4_roscon_25/transit_out/config/transit_out.yaml in the source workspace first,
# rebuild if you changed the file, or pass -p waypoints:=[lat,lon,...]
ros2 run transit_out transit_out --ros-args --params-file install/transit_out/share/transit_out/config/transit_out.yaml
```

Select **Transit Out** in QGroundControl after the node has registered. Heading is updated from the actual PX4 local NED ground velocity (`atan2(v_east, v_north)`), and the previous heading is retained while nearly stationary.
