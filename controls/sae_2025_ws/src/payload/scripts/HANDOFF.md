# Payload Handoff System

## Architecture

One `payload_agent.py` script runs two instances, differentiated by a `role` parameter. Each instance controls its own C++ payload node (motor driver).

```
payload_incoming (C++ motor driver) <- payload_incoming_agent (role=incoming)
                                              | /payload/agent_comm
payload_hooked   (C++ motor driver) <- payload_hooked_agent   (role=hooked)
```

## Related Files

| File | Description |
|------|-------------|
| `payload/scripts/payload_agent.py` | Unified Python agent (role=incoming or hooked) |
| `payload/launch/handoff.launch.py` | Launches all 4 nodes (2 C++ payloads + 2 Python agents) |
| `payload/src/payload.cpp` | C++ payload node (motor driver, listens on cmd_drive) |
| `payload/src/main.cpp` | C++ entry point |
| `payload/include/payload/payload.hpp` | Payload node header |
| `payload/include/payload/controller.hpp` | Abstract controller interface (GPIOController / SimController) |
| `payload/config/payload_params.yaml` | Shared parameters (pins, PID, kinematics, etc.) |
| `payload/CMakeLists.txt` | Build rules (installs payload_agent as executable) |
| `payload/package.xml` | Package dependencies |
| `payload_interfaces/src/msg/AgentComm.msg` | Peer-to-peer message (sender_name, message, timestamp) |
| `payload_interfaces/src/msg/PayloadMode.msg` | Mode broadcast (agent_name, mode with constants) |
| `payload_interfaces/src/msg/DriveCommand.msg` | Drive command (linear, angular) |
| `payload_interfaces/src/srv/PayloadCommand.srv` | Operator command service (command -> success, message) |

## Interfaces

| Interface | Who Sends | Who Receives | Purpose |
|-----------|-----------|-------------|---------|
| `AgentComm.msg` | agent <-> agent | The other agent | Peer-to-peer coordination (UNHOOK / CLEAR) |
| `PayloadMode.msg` | Both agents | Anyone | Broadcasts current state at 2 Hz for monitoring |
| `PayloadCommand.srv` | Operator | incoming agent only | Manual trigger for flow steps |

## Topics and Services

| Endpoint | Type | Purpose |
|----------|------|---------|
| `/payload_incoming/cmd_drive` | Topic | incoming agent drives its rover |
| `/payload_hooked/cmd_drive` | Topic | hooked agent drives its rover |
| `/payload/agent_comm` | Topic | Peer-to-peer messages between agents |
| `/payload/mode` | Topic | Both agents broadcast their current mode |
| `/payload_incoming/agent/command` | Service | Operator sends `start` or `near_plane` |

## Flow

1. Operator sends `start` to incoming agent
   - incoming: IDLE -> DRIVING_TO_PLANE, rover starts driving toward the plane
2. Operator sends `near_plane` to incoming agent
   - incoming: DRIVING_TO_PLANE -> NEAR_PLANE, rover stops
   - incoming sends `UNHOOK` over `/payload/agent_comm`
3. hooked agent receives `UNHOOK` (automatic)
   - hooked: HOOKED -> DRIVING_OFF, rover drives away
4. After 3 seconds hooked agent stops automatically
   - hooked: DRIVING_OFF -> CLEAR, sends `CLEAR` over `/payload/agent_comm`
5. incoming agent receives `CLEAR` (automatic)
   - incoming: NEAR_PLANE -> READY_FOR_PICKUP

## State Transitions

```
incoming:  IDLE -> DRIVING_TO_PLANE -> NEAR_PLANE -> READY_FOR_PICKUP
                ^ manual              ^ manual       ^ auto (on CLEAR)

hooked:    HOOKED -> DRIVING_OFF -> CLEAR
                  ^ auto (on UNHOOK) ^ auto (after 3s)
```

The operator only interacts with the incoming agent (2 commands). Everything else is automated through peer-to-peer messaging.

## How to Run

```bash
# Build (once)
cd ~/monorepo/controls/sae_2025_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select payload_interfaces --cmake-clean-first
source install/setup.bash
colcon build --packages-select payload
source install/setup.bash

# Terminal 1 -- launch all 4 nodes
ros2 launch payload handoff.launch.py domain_id:=5

# Terminal 2 -- step through the handoff
ros2 service call /payload_incoming/agent/command payload_interfaces/srv/PayloadCommand "{command: 'start'}"
ros2 service call /payload_incoming/agent/command payload_interfaces/srv/PayloadCommand "{command: 'near_plane'}"

# Terminal 3 -- monitor mode changes
ros2 topic echo /payload/mode
```
