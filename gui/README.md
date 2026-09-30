# 3D LiDAR GUI

This program subscribes to the existing brokerless ZeroMQ point-cloud stream and
draws the latest SICK multiScan frame in an interactive Qt window.

## Data flow

1. \`cloud_publisher_main\` receives UDP scan data from the SICK sensor.
2. \`SickCloudSource\` creates a \`lidar::PointCloud\`.
3. \`CloudPublisher\` serializes it with Protobuf and publishes two ZeroMQ frames:
   topic \`lidar.cloud\` and the serialized payload.
4. \`cloud_gui\` uses \`CloudSubscriber\` to receive and deserialize that message.
5. A background thread stores only the newest cloud.
6. The Qt GUI reads the newest cloud at about 30 display updates per second,
   converts polar points to x/y/z with \`lidar::to_xyz()\`, and draws them.

Keeping only the newest frame prevents an unbounded queue when rendering is
slower than the sensor.

## Install Qt on Ubuntu 22.04

\`\`\`bash
sudo apt update
sudo apt install qtbase5-dev libqt5opengl5-dev
\`\`\`

## Build

From the repository root:

\`\`\`bash
cmake -S . -B build
cmake --build build -j6
\`\`\`

The executable is:

\`\`\`text
build/gui/cloud_gui
\`\`\`

## Run the sensor publisher

Use one line; the backslashes only split it for readability:

\`\`\`bash
./build/apps/cloud_publisher_main \
  ~/sick_scan_ws/sick_scan_xd/launch/sick_multiscan.launch \
  192.168.12.223 \
  192.168.12.240 \
  "tcp://*:5580" \
  lidar.cloud \
  2125 \
  7513
\`\`\`

Meaning of the arguments:

| Argument | Meaning |
|---|---|
| launch file | SICK driver configuration |
| 192.168.12.223 | LiDAR IP address |
| 192.168.12.240 | computer Ethernet IP receiving sensor UDP |
| tcp://*:5580 | publisher binds port 5580 on all computer interfaces |
| lidar.cloud | topic name |
| 2125 | scan UDP port |
| 7513 | IMU UDP port |

## Run the GUI on the same computer

\`\`\`bash
./build/gui/cloud_gui
\`\`\`

Use:

- Endpoint: \`tcp://127.0.0.1:5580\`
- Topic: \`lidar.cloud\`
- Click **Connect**

## Run the GUI on another computer

Replace \`127.0.0.1\` with the publisher computer's Ethernet address:

\`\`\`text
tcp://192.168.12.240:5580
\`\`\`

Allow TCP port 5580 through the publisher computer firewall if necessary:

\`\`\`bash
sudo ufw allow 5580/tcp
\`\`\`

The LiDAR address is not entered in the GUI. The GUI connects to the middleware
publisher computer; the publisher is the process that connects to the LiDAR.

## Controls

- Left mouse drag: orbit around the cloud.
- Right mouse drag: pan.
- Mouse wheel: zoom.
- Front/Side/Top: fixed camera directions.
- Colour: height, intensity, or range.
- Minimum/maximum range: discard points outside the chosen distance.
- Pause display: freezes drawing while the receive thread continues keeping the
  latest frame.
- Clear view: removes the currently displayed points.

## Important source sections

### Receiver thread

\`toggleConnection()\` creates one \`std::thread\`. Inside that thread,
\`CloudSubscriber::poll_cloud(cloud, 100)\` waits for at most 100 ms. This keeps
the GUI thread free and makes disconnect finish quickly.

### Queue behavior

There is intentionally no growing queue. \`ReceiverState::latest\` is overwritten
whenever a new frame arrives. The mutex protects it while the receiver writes and
the GUI copies. Memory therefore stays approximately one current cloud plus one
display copy.

### Coordinate conversion

The middleware stores range, azimuth, and elevation. \`lidar::to_xyz()\` converts
each accepted sample using:

\`\`\`text
horizontal = range * cos(elevation)
x = horizontal * cos(azimuth)
y = horizontal * sin(azimuth)
z = range * sin(elevation)
\`\`\`

The convention is x forward, y left, and z up.

### Payload size

The GUI reports an estimated decoded payload size:

\`\`\`text
(number of float values * 4 bytes) + metadata
\`\`\`

It is an estimate because \`CloudSubscriber\` intentionally hides the serialized
Protobuf message. It is not the exact Ethernet packet size.
