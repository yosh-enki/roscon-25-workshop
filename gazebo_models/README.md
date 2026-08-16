# Gazebo Worlds (custom)

Additional Gazebo worlds placed in this folder (`gazebo_models/worlds/`), on
top of the stock PX4 worlds that live in `/home/ubuntu/PX4-gazebo-models/worlds/`.

Currently available:

| World | File | Notes |
|-------|------|-------|
| kmitl_airfield | `worlds/kmitl_airfield.sdf` | Custom airfield with ArUco landing pads (1..5), search area and checkpoints. |

> The ArUco pad textures are in `worlds/materials/textures/` and referenced
> with **relative** paths (`materials/textures/aruco_N.png`) so they resolve
> no matter where the world file is launched from.

## Workflow (run Gazebo first, then the drone)

This reproduces the standard "1) run gz, 2) run the drone" flow from
`docs/setup.md`, but using the custom world.

### Terminal 1 — start Gazebo with the custom world

```sh
# From the workspace root:
./gazebo_models/run_world.sh            # kmitl_airfield, with GUI
# or headless:
./gazebo_models/run_world.sh --headless
# or any other world in worlds/ :
./gazebo_models/run_world.sh kmitl_airfield
```

Alternatively (world must exist in `/home/ubuntu/PX4-gazebo-models/worlds/`):

```sh
python3 /home/ubuntu/PX4-gazebo-models/simulation-gazebo \
  --model_store /home/ubuntu/PX4-gazebo-models/ --world kmitl_airfield
```

`run_world.sh` sets `GZ_SIM_RESOURCE_PATH` to include the PX4 models directory,
so the drone model can be spawned afterwards. Ignore plugin load warnings.
You can also add `--headless` for servers without GUI.

### Terminal 2 — spawn the drone (x500 mono camera pointing down)

```sh
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4014 PX4_PARAM_UXRCE_DDS_SYNCT=0 \
  /home/ubuntu/px4_sitl/bin/px4 -w /home/ubuntu/px4_sitl/romfs
```

- `PX4_SYS_AUTOSTART=4014` is the airframe `4014_gz_x500_mono_cam_down`
  (x500 + downward-facing mono camera). Its init script sets the spawned model
  to `x500_mono_cam_down`.
- Other useful autostart ids: `4001` = plain `x500`, `4010` = `x500_mono_cam`.

### Terminal 3 — link the simulation to ROS 2

```sh
ros2 launch px4_roscon_25 common.launch.py
```

To also receive the downward camera images in ROS 2, bridge the `/camera` topic:

```sh
ros2 run ros_gz_bridge parameter_bridge /camera@image_transport/std_msgs/Image[gz.msgs.Image
```

## Notes

- The copy of `kmitl_airfield.sdf` + `materials/` under
  `/home/ubuntu/PX4-gazebo-models/worlds/` is only for convenience so the
  standard `simulation-gazebo --world kmitl_airfield` works in the current
  container; the canonical source lives in `gazebo_models/worlds/`.