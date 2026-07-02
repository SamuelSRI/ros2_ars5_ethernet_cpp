# ros2_ars5_ethernet_cpp

Experimental ROS 2 C++ Ethernet/SOME/IP driver for Continental ARS5-A radar sensors.

This package receives multicast UDP radar packets from Continental ARS5-A radars, decodes radar detections, publishes one `PointCloud2` topic per radar, and can optionally fuse all radar point clouds into a single fixed frame.

> This driver is experimental. The packet format was reverse-engineered from Ethernet captures and is not based on official Continental protocol documentation.

## Features

* Ethernet UDP multicast receiver
* SOME/IP packet validation
* Support for multiple Continental ARS5-A radars on the same multicast group
* Source IP based radar identification
* One `sensor_msgs/msg/PointCloud2` topic per radar
* Optional point cloud fusion node
* TF-based transformation into a fixed frame such as `base_link`
* RViz-compatible visualization
* Frame naming compatible with `robot_state_publisher`
* Designed for radar arrays connected through an Ethernet switch

## Tested setup

* Linux
* ROS 2 Lyrical
* Continental ARS5-A radar sensors
* Ethernet switch
* VLAN ID: `19`
* Multicast group: `224.0.2.2`
* Destination port: `42102`
* Radar source port: `42402`
* Observed radar IP range: `10.13.1.110` to `10.13.1.129`

## Network architecture

```text
Continental ARS5-A radars
        |
        | Ethernet / VLAN 19 / multicast UDP
        |
Ethernet switch
        |
        |
Linux PC / ROS 2 computer
        |
        | ROS 2 PointCloud2 topics
        |
RViz / perception / navigation
```

## ROS 2 nodes

This package contains two nodes:

```text
ars5_node
```

Receives radar packets and publishes one point cloud topic per radar.

```text
ars5_fusion_node
```

Subscribes to all radar point cloud topics, transforms them into a fixed frame using TF, and publishes a single fused point cloud.

## ROS 2 topics

### Per-radar topics

Each radar publishes its own topic:

```text
/radar/r110/points
/radar/r111/points
/radar/r112/points
...
/radar/r129/points
```

The radar ID comes from the last byte of the radar IP address.

Example:

```text
10.13.1.110 -> /radar/r110/points
10.13.1.117 -> /radar/r117/points
10.13.1.129 -> /radar/r129/points
```

### Fused topic

The fusion node publishes:

```text
/radar/points_fused
```

Default output frame:

```text
base_link
```

## Frames

Each radar topic uses a dedicated frame ID:

```text
/radar/r110/points -> radar_110_link
/radar/r111/points -> radar_111_link
/radar/r112/points -> radar_112_link
...
/radar/r129/points -> radar_129_link
```

The transforms from these radar frames to the fixed frame must be provided by your robot description / URDF and `robot_state_publisher`.

Example TF tree:

```text
base_link
├── radar_110_link
├── radar_111_link
├── radar_112_link
├── ...
└── radar_129_link
```

## PointCloud2 fields

Each point contains:

```text
x
y
z
velocity
range
azimuth
elevation
radar_id
```

The current decoder projects radar detections using range, azimuth and elevation.

## Current decoding assumptions

The observed ARS5-A packet format is:

```text
SOME/IP header: 16 bytes
Message ID:     0x00000150
Packet size:    35336 bytes
Payload length: 35328 bytes
```

Detection table:

```text
Start offset:   0x88
Stride:         44 bytes
Count:          800 slots
```

Currently decoded fields:

```text
azimuth   offset + 9
elevation offset + 18
range     offset + 26
velocity  offset + 34
```

All decoded values are interpreted as big-endian IEEE-754 floats.

## Limitations

* Experimental reverse-engineered decoder
* Not based on official Continental ARS5-A protocol documentation
* RCS is not decoded yet
* Detection validity flags are not fully decoded yet
* Quality/confidence fields are not decoded yet
* Elevation may require further validation depending on radar configuration
* Tested on a specific network setup: VLAN 19, multicast `224.0.2.2:42102`
* The CMake configuration was adapted for ROS 2 Lyrical
* Radar point clouds are sparse by nature and are not equivalent to LiDAR point clouds

## Dependencies

Install the required ROS 2 packages:

```bash
sudo apt update
sudo apt install ros-lyrical-rclcpp \
                 ros-lyrical-sensor-msgs \
                 ros-lyrical-std-msgs \
                 ros-lyrical-geometry-msgs \
                 ros-lyrical-tf2 \
                 ros-lyrical-tf2-ros \
                 ros-lyrical-tf2-msgs
```

## Build

Clone the package into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone git@github.com:SamuelSRI/ros2_ars5_ethernet_cpp.git
```

Build the package:

```bash
cd ~/ros2_ws

source /opt/ros/lyrical/setup.bash
colcon build --packages-select ros2_ars5_ethernet_cpp
source install/setup.bash
```

## Network setup

The radars were observed on VLAN 19. Create a VLAN interface before launching the node.

Replace `enp7s0` with your Ethernet interface name.

```bash
sudo ip link delete enp7s0.19 2>/dev/null

sudo ip addr flush dev enp7s0
sudo ip link add link enp7s0 name enp7s0.19 type vlan id 19
sudo ip addr add 10.13.1.100/24 dev enp7s0.19

sudo ip link set enp7s0 up
sudo ip link set enp7s0.19 up
```

Increase receive buffers:

```bash
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.rmem_default=134217728
sudo sysctl -w net.core.netdev_max_backlog=50000
```

Verify that radar packets are received:

```bash
sudo tcpdump -i enp7s0.19 -nn udp port 42102
```

Expected output:

```text
10.13.1.xxx.42402 > 224.0.2.2.42102: UDP, length 35336
```

## Configuration

### Radar receiver config

Default file:

```text
config/ars5.yaml
```

Example:

```yaml
ars5_node:
  ros__parameters:
    listen_ip: "0.0.0.0"
    listen_port: 42102
    multicast_group: "224.0.2.2"
    interface_ip: "10.13.1.100"
    max_packets_per_cycle: 64
```

Parameter meaning:

```text
listen_ip              Local bind IP
listen_port            UDP destination port
multicast_group        Multicast group used by the radars
interface_ip           IP address of the VLAN interface
max_packets_per_cycle  Maximum UDP packets processed per timer cycle
```

### Fusion node config

Default file:

```text
config/ars5_fusion.yaml
```

Example:

```yaml
ars5_fusion_node:
  ros__parameters:
    fixed_frame: "base_link"
    output_topic: "/radar/points_fused"
    radar_id_start: 110
    radar_id_end: 129
    publish_rate_hz: 10.0
    max_cloud_age_sec: 0.3
```

Parameter meaning:

```text
fixed_frame        Target frame used for fusion
output_topic       Output fused PointCloud2 topic
radar_id_start     First radar ID to subscribe to
radar_id_end       Last radar ID to subscribe to
publish_rate_hz    Fused cloud publication rate
max_cloud_age_sec  Maximum age of a radar cloud before it is ignored
```

## Launch radar receiver

Terminal 1:

```bash
cd ~/ros2_ws

source /opt/ros/lyrical/setup.bash
source install/setup.bash

ros2 launch ros2_ars5_ethernet_cpp ars5.launch.py
```

Expected log:

```text
ARS5 node listening on 0.0.0.0:42102 multicast=224.0.2.2 interface_ip=10.13.1.100
Created publisher for radar_id=117 on topic /radar/r117/points with frame radar_117_link
published X points on /radar/r117/points frame=radar_117_link
```

## Launch robot description

The fusion node requires valid TF transforms between the radar frames and the fixed frame.

Example:

```bash
ros2 launch your_robot_description your_robot_state_publisher.launch.py
```

Your URDF should contain frames such as:

```text
radar_110_link
radar_111_link
radar_112_link
...
radar_129_link
```

Example URDF link and joint:

```xml
<link name="radar_110_link"/>

<joint name="base_to_radar_110" type="fixed">
  <parent link="base_link"/>
  <child link="radar_110_link"/>
  <origin xyz="0.0 0.0 0.5" rpy="0 0 0"/>
</joint>
```

Repeat this for each radar with the correct position and orientation.

## Launch fusion node

Terminal 2:

```bash
cd ~/ros2_ws

source /opt/ros/lyrical/setup.bash
source install/setup.bash

ros2 launch ros2_ars5_ethernet_cpp ars5_fusion.launch.py
```

Expected output:

```text
Subscribed to /radar/r110/points
Subscribed to /radar/r111/points
...
ARS5 fusion node started. fixed_frame=base_link output=/radar/points_fused
published fused cloud with X points in frame base_link
```

## Check topics

List radar topics:

```bash
ros2 topic list | grep radar
```

Example output:

```text
/radar/r110/points
/radar/r111/points
/radar/r112/points
...
/radar/points_fused
```

Check publishing rate of one radar:

```bash
ros2 topic hz /radar/r117/points
```

Check fused topic rate:

```bash
ros2 topic hz /radar/points_fused
```

Inspect one fused message:

```bash
ros2 topic echo /radar/points_fused --once
```

## RViz visualization

Launch RViz:

```bash
rviz2
```

If your TF tree is correct, use:

```text
Fixed Frame: base_link
```

Then display the fused cloud:

```text
Add -> PointCloud2 -> /radar/points_fused
```

Recommended RViz settings:

```text
Style: Points
Size: 0.1 or 0.2
Decay Time: 0.5 to 2.0
```

You can also display individual radar clouds:

```text
/radar/r110/points
/radar/r111/points
/radar/r112/points
...
```

If the TFs are not ready yet, use one radar frame as the fixed frame:

```text
Fixed Frame: radar_110_link
```

Then display only:

```text
/radar/r110/points
```

## Useful TF commands

Check if a radar frame exists:

```bash
ros2 run tf2_ros tf2_echo base_link radar_117_link
```

If this command does not return a transform, the fusion node cannot use that radar cloud.

For a temporary static transform test:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link radar_117_link
```

## Troubleshooting

### No radar topics appear

Check if packets are received:

```bash
sudo tcpdump -i enp7s0.19 -nn udp port 42102
```

If no packets appear, check the VLAN interface:

```bash
ip addr show enp7s0.19
ip link show enp7s0.19
```

### Node starts but receives nothing

Make sure `interface_ip` in `config/ars5.yaml` matches the VLAN interface IP:

```yaml
interface_ip: "10.13.1.100"
```

### Fusion topic exists but has zero points

Check that the individual radar topics publish points:

```bash
ros2 topic hz /radar/r117/points
```

Check that TF exists:

```bash
ros2 run tf2_ros tf2_echo base_link radar_117_link
```

If TF is missing, update your URDF or launch `robot_state_publisher`.

### RViz says no transform

Example error:

```text
No transform from radar_117_link to base_link
```

This means the radar topic is working, but the TF is missing.

Add the radar link to your URDF and launch `robot_state_publisher`.

### Very few points are visible

Radar sensors do not produce dense point clouds like LiDAR. They usually publish sparse detections.

You can reduce filters in `src/ars5_decoder.cpp` if needed:

```cpp
if (range < 0.05f || range > 300.0f) {
  continue;
}
```

Be careful: reducing filters may show noisy or invalid detections.

### Walls are not detected

Automotive radars detect radar reflections, not visual surfaces. Flat walls may reflect energy away from the radar, especially if not perpendicular to the radar beam. Metallic objects, vehicles, poles, barriers and moving objects are usually detected better than flat indoor walls.

### A radar is unplugged

The node keeps running. The topic for that radar may still exist if the publisher was already created, but it will stop receiving new messages.

The fusion node ignores old clouds after:

```text
max_cloud_age_sec
```

Default:

```text
0.3 seconds
```

## Git commands

After modifying the package:

```bash
cd ~/ros2_ws/src/ros2_ars5_ethernet_cpp

git status
git add .
git commit -m "Update ARS5 radar driver"
git push
```

If the remote branch has different commits and you want to overwrite GitHub with your local version:

```bash
git push --force-with-lease origin main
```

## License

MIT
