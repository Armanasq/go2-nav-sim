# 13. Toward the real Go2

| | |
|---|---|
| Goal | Know what changes between this simulation and a real Go2 before you touch one: which robot you have, how to reach it on the network, which control level to use, the safety rules, the topic names, a tested `/cmd_vel` bridge, and how to run the chapter 9 estimator on recorded robot data. |
| Time | 2.5 h, no hardware needed. |
| Before you start | [Chapter 10](10-mapping-and-navigation.md) done; [chapter 12](12-troubleshooting.md) read, in particular the QoS rows. |
| You will learn | EDU vs AIR/PRO; the 192.168.123.x network and `CYCLONEDDS_URI`; sport mode vs `/lowcmd` and why you never mix them; Damp as the emergency stop; how this stack's topics map to the robot's; a bridge whose clamp and timeout logic is tested without ROS; the frame of the L1 lidar. |

No real Go2 was used for this chapter. Every hardware statement cites its section in
[`research/go2-real-robot.md`](../research/go2-real-robot.md), with the tag used there: **V** the source was opened and read,
**S** search snippet only, **U** not verified. Unitree's own developer pages (support.unitree.com) block automated clients and
were not read (**U**, §1.1). Check every **S** and **U** item in a browser before you rely on it. The commands below that run
without a robot were run here; the ones that need a robot are marked "not run".

## Task 1: Find out which Go2 you have

### 1.1 Read the model, firmware and lidar

Objective: write down the three facts that decide every later step.

| | EDU | AIR, PRO |
|---|---|---|
| Developer access | DDS over Ethernet, through `unitree_ros2` and `unitree_sdk2` (V §1.2, §1.3) | WebRTC over Wi-Fi, the phone app's protocol, through `go2_ros2_sdk` or `unitree_webrtc_connect` (V §1.6, §1.7) |
| Foot force sensors | yes (V §1.9) | no (V §1.9) |
| Rates over WebRTC | | lidar about 7 Hz, joint states 1 Hz on firmware 1.1.7 (V §1.6) |
| Firmware matters because | the sport interface needs EDU software 1.1.6 or later (S §1.1) | 1.1.15 and later need a per-device AES-128 key for the WebRTC handshake (V §1.6, §1.7) |

Record the lidar too. The product page now lists the L2 (360° × 96°), while most guides describe the L1 (360° × 90°, 11 Hz,
21,600 points/s) (V §1.9). Units in the field may carry either (my inference, §1.9).

This chapter follows the EDU path: it is the one where this course's ROS 2 nodes talk to the robot directly.

Check: you have the model, the firmware version and the lidar model written down. Not run: no robot.

## Task 2: Reach the robot on the network

### 2.1 Give the PC a static address

Objective: a wired link to the robot's 192.168.123.0/24 network.

The Go2 has no DHCP server, so the PC needs a static address (V §1.11). Unitree uses 192.168.123.99/24 (V §1.2); labs also use
.100 (V §1.8) and .222 (V §1.11). Never take 192.168.123.161 or 192.168.123.18. The ACRE Lab wiki gives .161 as the onboard computer
and .18 as the Jetson (V §1.12, §1.11); that .161 is the motion-control board comes from search snippets only (S §1.12). Not run: no robot.

```bash
sudo nmcli con add type ethernet ifname enp3s0 con-name go2 ipv4.method manual ipv4.addresses 192.168.123.99/24
sudo nmcli con up go2
ping -c 3 192.168.123.161
```

Replace `enp3s0` with your wired interface (`ip -br link`).

Check: the ping answers. Over Wi-Fi, `ros2 topic echo` can hang because point clouds saturate the link (V §1.11); map on Ethernet.

### 2.2 Point Cyclone DDS at that interface

Objective: ROS 2 traffic on the robot's network, in the robot's DDS domain.

The robot runs Cyclone DDS 0.10.2; Humble's `rmw_cyclonedds_cpp` works with it without a source build, which is why Unitree
recommends Humble (V §1.2). This is `unitree_ros2`'s `setup.sh` without its hard-coded Foxy path (V §1.2):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name="enp3s0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
ros2 daemon stop; ros2 daemon start
```

The robot stays on domain 0 (V §1.11). Keep the simulator on another domain so a forgotten simulator cannot command the robot.
`unitree_ros2`'s `setup_local.sh` does the same with `lo` for simulation (V §1.2).

What the two outcomes look like, run on this PC. With `lo`, which exists and has an address:

```
ros2: selected interface "lo" is not multicast-capable: disabling multicast
/parameter_events
/rosout
```

With `enp3s0` while its cable is unplugged (`ip -br link` shows `NO-CARRIER`):

```
ros2: enp3s0: does not match an available interface.
[ERROR] [...] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Error
```

Check: `ros2 topic list` runs without the `failed to create domain` error. If you see it, the interface in `CYCLONEDDS_URI` is
wrong, down or has no address.

### 2.3 See the robot's state

Objective: one message from the robot before anything moves.

```bash
ros2 topic echo --once /sportmodestate --field position
```

Not run: no robot. `sportmodestate` carries mode, gait type, position, velocity, yaw speed, body height, foot forces and an IMU
state (V §1.2); the frame of `position` is not documented in what I could read (U, §1.1). If nothing arrives, check QoS first
([chapter 12](12-troubleshooting.md) Task 1). The sources disagree: `unitree_ros2`'s open PR #29 gives telemetry subscribers
best-effort (sensor-data) QoS (V §1.2), while Southampton's guide says to use Reliable on its driver's `/scan` if nothing arrives
(V §1.11). Subscribe Best Effort: a Best Effort subscriber matches a publisher of either kind, a Reliable one only a Reliable
publisher.

Check: one message prints. Do not continue to Task 3 until it does.

## Task 3: Use one control level at a time

### 3.1 Choose high level for everything in this course

Objective: know the two ways to command the robot and the rule between them.

| | High level (sport mode) | Low level |
|---|---|---|
| Topic and type | `/api/sport/request`, `unitree_api/msg/Request`; replies on `/api/sport/response` (V §1.2) | `/lowcmd`, `unitree_go/msg/LowCmd` (V §1.2) |
| What you send | an API ID and JSON parameters: Damp 1001, BalanceStand 1002, StopMove 1003, StandUp 1004, StandDown 1005, RecoveryStand 1006, Move 1008, SwitchAvoidMode 2058 (V §1.2) | per motor q, dq, tau, kp and kd, plus a CRC (V §1.2) |
| Who balances | Unitree's controller | you |
| Use it for | walking, Nav2, this course | joint-level research only |

Never send low-level commands while the built-in motion service runs: both would command the same motors. Unitree's low-level
example switches the motion service off first (`MotionSwitcherClient::ReleaseMode()`, or in the app) (V §1.3, §1.4). Then, and only
then:

- the robot is hung up or lying on the ground; the SDK example prints exactly this warning and waits for Enter (V §1.3);
- every motor starts passive (`q = PosStopF`, `kp = 0`) and ramps to a low gain; Unitree's examples use kp 5 to 10 and kd 1 (V §1.3, §1.4);
- `sportmodestate` stops publishing on the real robot once sport mode is off, although `unitree_mujoco` keeps publishing it (V §1.5).

Check: you can name the topic each level uses. Everything later in this chapter uses `/api/sport/request` only.

## Task 4: Read the safety page before any motion

### 4.1 Go through the checklist

Objective: a checklist you read aloud before the robot moves.

- The area is clear and no one is within reach. `/cmd_vel` on the robot has no collision avoidance (V §1.11).
- One person holds the Unitree remote the whole time.
- Damp (1001) is the emergency stop (V §1.11). Map your own stop key to it.
- StandDown (1005) before power-off (V §1.11).
- No flips and no "wallow" (V §1.11).
- Built-in obstacle avoidance is a separate switch (SwitchAvoidMode 2058, `ObstaclesAvoidClient`). Do not assume it is on (V §1.2).
- Follow the robot on the first autonomous runs; pick it up when it is about to do something silly (V §1.6).
- Bluetooth off and the robot on an isolated network: the UniPwn exploit gains root through the BLE Wi-Fi setup and affects the
  Go2 (V §1.15). DDS has no authentication, so any host on the robot's network can publish `/lowcmd`, and the SDK does not clamp
  torque (V §1.15).

Check: before each session, the remote is in a hand and the Damp key is tested with the robot lying down. Not run: no robot.

## Task 5: Map the simulator's topics to the robot's

### 5.1 Read the topic map

Objective: know what replaces each topic this stack uses.

| This stack (sim) | Real Go2 EDU | Note |
|---|---|---|
| `/cmd_vel` (`Twist`) → CHAMP | `/api/sport/request`, Move 1008 | no merged `Twist` bridge in `unitree_ros2`; PRs #17 and #21 were still open on 2026-11-28 (V §1.2, rechecked); 5.2 is one |
| `/lidar/points`, frame `lidar_link` | `utlidar/cloud`, frame `utlidar_lidar` | mounted inverted and tilted; CMU rotates it by pitch 2.8782 rad (V §1.8) |
| `/imu/data` | `utlidar/imu` (in the lidar frame), or the IMU in `sportmodestate` and `lowstate` | the L1's IMU needs a one-time calibration (V §1.8) |
| `/joint_states` | `lowstate`, `motor_state` (q, dq, tau_est) | (V §1.2) |
| `/camera/image_raw` | an H.264 stream | needs a republisher to `sensor_msgs/Image` (V §1.8) |
| `/odom` from the EKF; `/odom_gt` | `sportmodestate` position and velocity; no ground truth | the robot publishes no tf (V §1.2) |
| TF from `robot_state_publisher` and the EKF | none | you publish `odom` → `base_link` and the lidar mount yourself (V §1.2) |

Check: for each row you know which node of this stack would have to change. `scan.yaml`'s height band (chapter 10, 5.2) also
assumes the simulated lidar's mount; it has to be redone for `utlidar_lidar`.

### 5.2 Write the cmd_vel bridge

Objective: Nav2's `/cmd_vel` turned into sport-mode Move requests, clamped, and stopped when commands stop.

`src/go2_tutorials/go2_tutorials/sport_bridge.py`. The decision is a pure function, `decide()`, so it can be tested without ROS
or the robot. `main()` is the ROS side.

```python
API_MOVE, API_STOPMOVE = 1008, 1003
LIMITS = (0.3, 0.25, 0.5)  # vx, vy (m/s), vyaw (rad/s): this course's CHAMP limits; far inside the Move range (that range is S)
TIMEOUT = 0.5              # s without /cmd_vel before StopMove; the robot itself holds a Move for about 1 s (S)


def decide(cmd, age, stopped, limits=LIMITS, timeout=TIMEOUT):
    """cmd = last (vx, vy, vyaw) or None; age = s since it arrived; stopped = StopMove already sent.
    Returns ('move', clamped cmd), ('stop', None), or (None, None) when there is nothing to send."""
    if cmd is None or age > timeout or not all(math.isfinite(v) for v in cmd):
        return (None, None) if stopped else ('stop', None)
    return 'move', tuple(max(-lim, min(lim, v)) for v, lim in zip(cmd, limits))


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from unitree_api.msg import Request   # from unitree_ros2 (cyclonedds_ws); not part of this course's install

    rclpy.init()
    node = rclpy.create_node('sport_bridge')
    pub = node.create_publisher(Request, '/api/sport/request', 10)
    st = {'cmd': None, 't': 0.0, 'stopped': True}
    now = lambda: node.get_clock().now().nanoseconds * 1e-9

    def on_twist(m):
        st['cmd'], st['t'] = (m.linear.x, m.linear.y, m.angular.z), now()

    def tick():
        kind, v = decide(st['cmd'], now() - st['t'], st['stopped'])
        if kind is None:
            return
        req = Request()
        req.header.identity.api_id = API_MOVE if kind == 'move' else API_STOPMOVE
        if v:
            req.parameter = json.dumps({'x': v[0], 'y': v[1], 'z': v[2]})
        pub.publish(req)
        st['stopped'] = kind == 'stop'

    node.create_subscription(Twist, '/cmd_vel', on_twist, 10)
    node.create_timer(0.05, tick)   # 20 Hz
    rclpy.spin(node)
```

What it targets, and how sure that is:

- The message, topic and API IDs are from `unitree_ros2`: `example/src/include/common/ros2_sport_client.h` (Move 1008, StopMove
  1003, publisher on `/api/sport/request`) and `ros2_sport_client.cpp`, whose `Move()` writes `{"x": vx, "y": vy, "z": vyaw}` into
  `parameter` and sets `header.identity.api_id` (V §1.2; both files read at commit 668d1ec for this chapter).
- The Move ranges (vx about −2.5 to 3.8 m/s, vy −1 to 1, vyaw −4 to 4) and the roughly 1 s that a Move stays in effect are **S**
  (§1.1). `LIMITS` are this course's CHAMP limits (0.3 m/s forward, 0.25 m/s sideways, 0.5 rad/s), far inside them, and the 0.5 s
timeout stops the robot before its own hold would run out.
- The ROS side was never run against a robot. It cannot run here: `unitree_api` is not installed.

```bash
cd src/go2_tutorials && python3 -m go2_tutorials.sport_bridge
```

```
ModuleNotFoundError: No module named 'unitree_api'
```

The logic is tested without ROS (`test/test_sport_bridge.py`: clamping on each axis, one StopMove on timeout and none after it,
nothing before the first command, and a stop on NaN or infinity):

```bash
cd src/go2_tutorials && python3 -m pytest -q test/test_sport_bridge.py
```

```
....                                                                     [100%]
4 passed in 0.49s
```

Check: the 4 tests pass. With `TIMEOUT` changed to 2.0, `test_timeout_sends_one_stop` fails (run here: `1 failed, 3 passed`), so the test does watch the timeout.

Common mistakes: `unitree_mujoco` cannot test this bridge. It simulates only the low-level interface (`LowCmd`, `LowState`,
`SportModeState`) and has no sport-mode service (V §1.5). The course plan (`research/gap-plan.md`) asked for a `unitree_mujoco` test of the
bridge; this chapter drops it on purpose for that reason and tests the logic with the unit tests above instead. The first test of the ROS side is on the real robot, lying down or hung
up, with the remote in hand and a single 0.1 m/s command.

## Task 6: Replay the chapter 9 estimator on robot data

### 6.1 Record on the robot

Objective: a bag of the robot's own lidar and IMU, to test the estimator before the robot walks under autonomy.

```bash
ros2 bag record /utlidar/cloud /utlidar/imu /sportmodestate
```

Not run: no robot. This is CMU's recording command plus the robot's state (V §1.8). Drive with the remote while recording. I found
no public Go2 bag of these topics, so there is nothing to download in place of this step (U).

### 6.2 Publish the L1's mount

Objective: a transform from `base_link` to `utlidar_lidar`, which the robot does not publish.

```bash
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id utlidar_lidar --pitch 2.8782 &
ros2 run tf2_ros tf2_echo base_link utlidar_lidar
```

```
At time 0.0
- Translation: [0.000, 0.000, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.991, 0.000, 0.131]
- Rotation: in RPY (radian) [3.142, 0.263, 3.142]
- Rotation: in RPY (degree) [180.000, 15.091, 180.000]
- Matrix:
 -0.966  0.000  0.260  0.000
  0.000  1.000  0.000  0.000
 -0.260  0.000 -0.966  0.000
  0.000  0.000  0.000  1.000
```

`tf2_echo` repeats this block once per second; stop it with Ctrl+C.

The pitch is CMU's (V §1.8); the translation is left at zero because neither source in my notes gives it. tf2 prints the same
rotation as roll 180°, pitch 15.1°, yaw 180°: the lidar is upside down and tilted by 15°.

Check: the lidar's z axis points down in `base_link`. The columns of the 3 × 3 rotation block are the lidar's axes in `base_link`,
so its z axis is the third column, (0.260, 0, −0.966): mostly −z, tilted forward by 15°. Then check it in RViz: with Fixed Frame `base_link`, the floor in `utlidar/cloud` must lie flat under the robot.

### 6.3 Replay with the robot's topic names

Objective: the chapter 9 lidar odometry reading a bag whose topics have the robot's names.

`lidar_odom.py` subscribes to `lidar/points` and `imu/data`, so remap them. Here the same command runs on the simulator bag `bags/w2`,
renamed on playback to the robot's names, to check the remapping:

```bash
ros2 run go2_nav_sim lidar_odom.py --ros-args -r lidar/points:=/utlidar/cloud -r imu/data:=/utlidar/imu -p use_sim_time:=true &
ros2 bag play bags/w2 --clock --topics /lidar/points /imu/data /tf /tf_static \
  --remap /lidar/points:=/utlidar/cloud /imu/data:=/utlidar/imu
```

```
[lidar_odom]: #50 pose=(0.02,-0.03,0deg) matches=282 rms=0.029 map=273 fails=0
[lidar_odom]: #100 pose=(-0.11,-0.06,22deg) matches=320 rms=0.043 map=380 fails=0
...
[lidar_odom]: #250 pose=(1.59,2.61,44deg) matches=250 rms=0.034 map=631 fails=0
```

On a robot bag, play only `/utlidar/cloud` and `/utlidar/imu`, keep 6.2's transform running, and add
`-p lidar_frame:=utlidar_lidar -p base_frame:=base_link`. The estimator's height band (0.30 m, [chapter 10](10-mapping-and-navigation.md)
5.2) is measured from this simulator's `base_footprint`; set it again for your base frame. Disconnect from the robot first, or its live topics mix with the bag
(V §1.8). Robot stamps are not host time; CMU shifts them by a one-off offset (V §1.8).

`lidar_odom.py` reads the gyro's z axis and the IMU's roll and pitch straight from the message, without TF. In simulation the IMU
is level in the trunk. The L1's IMU is in the inverted lidar frame, so its z rate has the opposite sign and its roll and pitch
are wrong (my inference from the code and 6.2). Rotate `utlidar/imu` into `base_link` before the estimator, as CMU's
`transform_everything.py` does (V §1.8).

Check: `pose=` lines appear and `fails=0`. On robot data, turn the robot left by hand during recording: the replayed yaw must increase.

## Exercise

The bridge has been forwarding `/cmd_vel` at 0.8 m/s forward. Nav2 crashes and the topic goes silent. What does the bridge send in
the next second, and what would happen if `decide()` had no `age > timeout` check?

Answer: `decide()` clamps 0.8 to 0.3 m/s, so the bridge keeps sending Move(0.3, 0, 0) at 20 Hz while the last command is at most
0.5 s old, then one StopMove (1003), then nothing (`test_timeout_sends_one_stop`). Without the age check, the 20 Hz timer would
resend the last command, Move(0.3, 0, 0), forever. The robot's own hold of about 1 s (S §1.1) never runs out, because a new Move
arrives every 0.05 s, so the robot walks on at 0.3 m/s until someone uses the remote or Damp.

## Further reading

- `unitreerobotics/unitree_ros2`: https://github.com/unitreerobotics/unitree_ros2
- `abizovnuralem/go2_ros2_sdk`, slam_toolbox and Nav2 on the real robot: https://github.com/abizovnuralem/go2_ros2_sdk
- Southampton robot guides, Go2 ROS control: https://robots.ecs.soton.ac.uk/go2/ros_control
- Context for UniPwn: https://arxiv.org/abs/2509.14139 (the paper studies Unitree's humanoid; the Go2 claim is from §1.15)
