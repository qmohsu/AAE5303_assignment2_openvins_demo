# Docker Container Guide for OpenVINS Demo

This guide explains how to run the OpenVINS demo container.

## 🐳 Container Information

| Property | Value |
|----------|-------|
| **Saved Image** | `ros2_openvins_configured:latest` |
| **Original Image** | `liangyu99/ros2_humble_dev:20251205` |
| **Container Name** | `ros2_humble_openvins` |

---

## 🚀 Quick Start (Existing Container)

### Option 1: Restart the Existing Container (Fastest)

If the container `ros2_humble_openvins` already exists:

```bash
# Check if container exists
docker ps -a | grep ros2_humble_openvins

# Start the existing container
docker start ros2_humble_openvins

# Enter the container
docker exec -it ros2_humble_openvins bash
```

### Option 2: Run from Saved Image (New Container)

If you need a fresh container from the saved image:

```bash
docker run -dit \
  --name ros2_openvins_new \
  --privileged \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /mnt/wslg:/mnt/wslg \
  -v /home/litahsu/AAE5303_assignment2_openvins_demo:/root/workspace \
  ros2_openvins_configured:latest
```

---

## 📋 Step-by-Step Guide to Run OpenVINS

### Step 1: Start the Container

```bash
# Start existing container
docker start ros2_humble_openvins

# Or create new one from saved image
docker run -dit \
  --name ros2_humble_openvins \
  --privileged \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /mnt/wslg:/mnt/wslg \
  -v /home/litahsu/AAE5303_assignment2_openvins_demo:/root/workspace \
  ros2_openvins_configured:latest
```

### Step 2: Enter the Container

```bash
docker exec -it ros2_humble_openvins bash
```

### Step 3: Source ROS2 Environment (Inside Container)

```bash
source /opt/ros/humble/setup.bash
source /home/open_vins_ws/install/setup.bash
```

### Step 4: Start Image Decompressor (Terminal 1)

```bash
ros2 run image_transport republish compressed raw \
  --ros-args \
  -r in/compressed:=/left_camera/image/compressed \
  -r out:=/left_camera/image_raw
```

### Step 5: Launch OpenVINS (Terminal 2)

Open another terminal and enter container:
```bash
docker exec -it ros2_humble_openvins bash
source /opt/ros/humble/setup.bash
source /home/open_vins_ws/install/setup.bash

ros2 launch ov_msckf subscribe.launch.py \
  config:=hkairport \
  max_cameras:=1 \
  use_stereo:=false
```

### Step 6: Launch RVIZ (Terminal 3)

```bash
docker exec -it ros2_humble_openvins bash
source /opt/ros/humble/setup.bash
source /home/open_vins_ws/install/setup.bash

rviz2 -d /home/open_vins/config/hkairport/display.rviz
```

### Step 7: Play ROS Bag (Terminal 4)

```bash
docker exec -it ros2_humble_openvins bash
source /opt/ros/humble/setup.bash

ros2 bag play /home/HKairport_GNSS03_ros2 --rate 0.5
```

---

## 🛑 Stop and Clean Up

### Stop Container (Keep Data)

```bash
docker stop ros2_humble_openvins
```

### Remove Container (Delete)

```bash
docker rm ros2_humble_openvins
```

### Remove Saved Image

```bash
docker rmi ros2_openvins_configured:latest
```

---

## 📁 Important Paths Inside Container

| Path | Description |
|------|-------------|
| `/home/open_vins/` | OpenVINS source code |
| `/home/open_vins_ws/` | OpenVINS compiled workspace |
| `/home/HKairport_GNSS03_ros2/` | ROS2 bag file |
| `/home/open_vins/config/hkairport/` | HKairport configuration |
| `/root/workspace/` | Mounted evaluation scripts |

---

## 🔧 Troubleshooting

### Container Not Starting

```bash
# Check if container exists
docker ps -a

# Remove and recreate if needed
docker rm ros2_humble_openvins
# Then run the docker run command again
```

### GUI Not Working

```bash
# On host machine (outside container)
xhost +local:docker

# Verify DISPLAY variable
echo $DISPLAY
```

### OpenVINS Not Initializing

- Ensure image decompressor is running first
- Check bag is playing: `ros2 topic hz /left_camera/image_raw`
- Verify IMU data: `ros2 topic hz /dji_osdk_ros/imu`

---

## 💾 Save Container Changes

If you make changes inside the container and want to save them:

```bash
# Commit container to new image
docker commit ros2_humble_openvins ros2_openvins_configured:v2

# Or overwrite existing
docker commit ros2_humble_openvins ros2_openvins_configured:latest
```

---

## 📤 Export/Import Image

### Export to File

```bash
docker save ros2_openvins_configured:latest | gzip > ros2_openvins_configured.tar.gz
```

### Import from File

```bash
gunzip -c ros2_openvins_configured.tar.gz | docker load
```

