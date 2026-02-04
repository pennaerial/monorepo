# Payload Package
Payload node responsible for receiving and converting drive commands to movement (in real and sim)

Payload for SAE Advanced Class 2026
- two wheel diff drive payload
- cam mounted in front
- ball caster in back

### Extra Dependencies
`sudo apt install ros-humble-generate-parameter-library`
- uses `generate_parameter_library` to load in node parameters as C++ structs


If running multiple instances of payload, they should differ by launch arguments, but share the same node parameters

### Launch configuration:
The defined launch arguments are:
- `payload_name`: node name of the payload to launch. The node name is used to determine the name of the ros topics that the payload node listens to, and also the gazebo topics/services that the node sends messages to (sim controller). **This must match the name of the entity name of the gazebo payload model (sim mode)** \
DEFAULT VALUE: `payload_0`

### Payload Parameters
Defined payload parameters (defined in `config/payload_params.schema.yaml`):
- controller: defines the controller to use. The available options are `sim` and `gpio`. Use `sim` when testing in sitl and `gpio` when running on the pi \
DEFAULT VALUE: `sim`

### Launch:
Running the launch file will run one instance of the payload and also links payload_params
from ws directory, run 

```bash
ros2 launch payload payload.launch.py
```
- this will launch the payload node with the params found in `config/payload_params.yaml`
- this will also use the default launch configuration

You can override the launch configuration via command line \
Example: overriding the `payload_name` launch argument:
```bash
ros2 launch payload payload.launch.py payload_name:=payload_1
```
- This will launch a payload node called with name `payload_1`
- This will listen to a ros topic called `/payload_1/cmd_drive` for drive commands
- This will publish drive commands to a gazebo topic called `/model/payload_1/cmd_vel` to control the sim payload 



