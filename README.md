# RB2301 Tutorial Package

A comprehensive ROS2 learning package demonstrating core concepts through multiple tutorial nodes.

---

## Project Overview

This package contains 6 tutorial nodes that showcase different ROS2 concepts and patterns:

| Node | Purpose | ROS2 Concepts |
|------|---------|---------------|
| **tutorial** | Turtle control with turtlesim | Publishers, Subscribers, Service Clients, Timers |
| **logger** | Logging at different severity levels | Logging API, Timers |
| **publishers** | Demonstrate QoS profiles for publishing | Publishers, QoS Profiles, Message Timestamps |
| **subscribers** | Demonstrate QoS profiles for subscribing | Subscribers, QoS Profiles, Time/Duration |
| **recorder** | Record turtle position data to file | Subscribers, File I/O |
| **fake** | Placeholder node | Basic ROS2 setup |

---

## Node Details

### 1. **Tutorial Node** (`tutorial.py`)

**Purpose**: Controls a turtle in turtlesim, spawns new turtles, and demonstrates basic node interactions.

**ROS2 Concepts Used**:
- **Timers**: `create_timer(0.5, callback)` - Periodic callback execution every 500ms
- **Publishers**: `create_publisher(Twist, '/turtle1/cmd_vel', 10)` - Send velocity commands to turtle
- **Subscribers**: `create_subscription(Pose, '/turtle1/pose', callback, 10)` - Listen to turtle's position
- **Service Clients**: `create_client(Spawn, '/spawn')` - Request to spawn new turtles asynchronously
- **Futures**: `future.done()`, `future.result()` - Asynchronous service call handling

**Functionality**:
- Publishes alternating forward/backward velocity commands (1.0 and -1.0)
- Subscribes to turtle's current pose and prints it every 0.5 seconds
- Makes an asynchronous service call to spawn a new turtle at position (1.0, 1.0)
- Displays the spawned turtle's name when the service completes

---

### 2. **Logger Node** (`logger.py`)

**Purpose**: Demonstrates ROS2's logging system at different severity levels.

**ROS2 Concepts Used**:
- **Timers**: `create_timer(0.5, callback)` - Periodic logging every 500ms
- **Logging API**: 
  - `get_logger().debug()` - Debug level messages
  - `get_logger().info()` - Info level messages
  - `get_logger().warn()` - Warning level messages
  - `get_logger().error()` - Error level messages
- **Logger Level Configuration**: `set_logger_level()` - Set logging severity threshold (set to WARN in this example)

**Functionality**:
- Increments a counter every 0.5 seconds
- Logs messages at all severity levels (debug, info, warn, error)
- Logger level is set to WARN, so only warnings and errors are displayed (debug and info are filtered)

---

### 3. **Publishers Node** (`publishers.py`)

**Purpose**: Demonstrates different QoS (Quality of Service) profiles for publishing.

**ROS2 Concepts Used**:
- **Publishers**: Creates two publishers with different QoS profiles
- **QoS Profiles**: 
  - **Sensor Data QoS**: Best-effort delivery, optimized for real-time sensor streams
  - **Latch/Transient Local QoS**: History policy with TRANSIENT_LOCAL durability (messages persist for new subscribers)
- **Message Timestamps**: `get_clock().now().to_msg()` - Attach ROS2 timestamps to messages
- **Timers**: `create_timer(0.05, callback)` - Fast publish rate (20Hz)

**Functionality**:
- Publishes `Header` messages with timestamps and incrementing frame_id
- Publishes to two topics:
  - `/sensor` - Uses sensor data QoS (optimized for high-frequency data)
  - `/latch` - Uses latch QoS (messages retained for new subscribers)
- Demonstrates how QoS profiles affect message delivery and retention

---

### 4. **Subscribers Node** (`subscribers.py`)

**Purpose**: Demonstrates different QoS profiles for subscribing and message lifespan tracking.

**ROS2 Concepts Used**:
- **Subscribers**: Subscribe to multiple topics with different QoS profiles
- **QoS Profiles**: 
  - **Sensor Data QoS**: Matches sensor data publishing
  - **Latch QoS**: Matches the latch publishing profile
- **Time and Duration**: 
  - `get_clock().now()` - Get current ROS2 time
  - `Time.from_msg(msg.stamp)` - Convert message timestamp
  - `Duration(seconds=...)` - Create time durations
  - `get_clock().sleep_for()` - Sleep for specified duration
- **Message Timestamps Analysis**: Calculates elapsed time since message was published

**Functionality**:
- Subscribes to `/sensor` and `/latch` topics with appropriate QoS profiles
- Stores received messages and displays them every 0.1 seconds
- Calculates and displays how long each message has been "alive" (elapsed time)
- Demonstrates how different QoS profiles and durability settings affect message retention
- Can simulate blocking callbacks by sleeping during timer callback

---

### 5. **Recorder Node** (`recorder.py`)

**Purpose**: Records turtle position data to a file for later analysis.

**ROS2 Concepts Used**:
- **Subscribers**: `create_subscription(Pose, '/turtle1/pose', callback, 10)` - Listen to turtle positions
- **Callbacks**: `sub_callback()` - Custom callback for message handling
- **File I/O**: Write received data to a text file (data.txt)

**Functionality**:
- Subscribes to `/turtle1/pose` topic to receive turtle position updates
- Records position data (x, y, theta) to `data.txt` with tab-separated format
- First row contains headers: `x`, `y`, `theta`
- Each subsequent row contains the turtle's position and orientation angle

---

### 6. **Fake Node** (`fake.py`)

**Purpose**: A minimal placeholder node for testing or basic ROS2 setup.

**ROS2 Concepts Used**:
- Basic node structure and lifecycle (init, spin, shutdown)

**Functionality**:
- Prints "Hi from fake" when executed
- Serves as a template for new nodes

---

## How to Use

### Build the package:
```bash
cd ~/rb2301
colcon build
```

### Source the setup file:
```bash
source ~/rb2301/install/setup.bash
```

### Run individual nodes:
```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Run tutorial node
ros2 run rb2301_tutorial tutorial

# Or run other nodes
ros2 run rb2301_tutorial logger
ros2 run rb2301_tutorial publishers
ros2 run rb2301_tutorial subscribers
ros2 run rb2301_tutorial recorder
```

### View recorded data:
```bash
cat ~/rb2301/data.txt
```

---

## Key ROS2 Concepts Summary

| Concept | Nodes Using It | Purpose |
|---------|----------------|---------| 
| **Timers** | tutorial, logger, publishers, subscribers | Execute callbacks periodically |
| **Publishers** | tutorial, publishers | Send messages to topics |
| **Subscribers** | tutorial, subscribers, recorder | Receive messages from topics |
| **Service Clients** | tutorial | Request-response pattern with services |
| **QoS Profiles** | publishers, subscribers | Control message delivery quality and retention |
| **Logging** | logger | Output diagnostic information at various levels |
| **Time/Duration** | subscribers | Work with ROS2 time and measure elapsed time |
| **Message Callbacks** | All nodes | Handle incoming messages and timer events |
| **Futures** | tutorial | Handle asynchronous service responses |