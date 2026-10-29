# Dev container

ROS 2 Jazzy (`ros:jazzy-ros-base`) with Cyclone DDS and every dependency declared in the
`package.xml` files under `src/`.

## Image

- Stage `manifests` copies `src/` and keeps only `package.xml` and `COLCON_IGNORE` /
  `AMENT_IGNORE` / `CATKIN_IGNORE` files.
- The main stage runs `rosdep install` on that result. The rosdep layer is rebuilt only when a
  manifest changes, not on code changes. Unresolvable keys fail the build.
- The build context is the repo root. `.dockerignore` lets in only `src/`.

After adding a dependency to a `package.xml`, rebuild the container, or install it without a
rebuild:

```bash
rosdep install --from-paths src --ignore-src -y
```

## Runtime

- The repo is bind-mounted at `/robotarm_firmware_lnx` (the workspace folder), so `build/`,
  `install/` and `log/` stay on the host.
- Hardware and real-time: `--privileged`, `/dev` and `/sys` mounts, `/dev/ttyACM0` (fdcanusb,
  must be plugged in), `sys_nice`, `rtprio=99`, `memlock=-1`. See
  [docs/REALTIME_SETUP.md](../docs/REALTIME_SETUP.md).
- `--net=host` and `--ipc=host` for DDS.
- X11 socket and `DISPLAY` are forwarded for MuJoCo and RViz.
- `~/.bashrc` sources `/opt/ros/jazzy`. Source `install/setup.bash` yourself after building
  (the VS Code tasks do this).
