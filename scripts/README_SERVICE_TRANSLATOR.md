# Spot Service Translator

This package provides a mechanism to translate ROS2 services into topic interfaces, allowing non-blocking command execution.

## Gripper Angle Translator

### Overview
The gripper angle translator converts topic messages on `/spot/gripper_angle_cmd` into service calls to `/spot/set_gripper_angle`.

### Usage

#### Launch the Translator
```bash
ros2 launch spot_multi spot_service_translate.launch.py
```

With custom spot name:
```bash
ros2 launch spot_multi spot_service_translate.launch.py spot_name:=spot2
```

#### Send Gripper Commands via Topic
```bash
# Set gripper angle to 45 degrees (range: 0-90)
ros2 topic pub /spot/gripper_angle_cmd std_msgs/msg/Float32 "data: 45.0"
```

### Parameters
- `spot_name`: Name of the Spot robot (default: "spot")
- `update_rate`: Rate in seconds to check for new requests (default: 0.1)

### How It Works
1. The translator subscribes to `/{spot_name}/gripper_angle_cmd` topic
2. Periodically checks if a new gripper angle request has been received
3. If a new value is detected, it calls the `/{spot_name}/set_gripper_angle` service asynchronously
4. This prevents blocking behavior when controlling the gripper

### Creating Custom Translators

To create a translator for other services, use the `SpotServiceTranslator` class:

```python
from spot_service_translator import SpotServiceTranslator
from your_msgs.srv import YourService
from std_msgs.msg import YourMsgType

def build_request(msg: YourMsgType) -> YourService.Request:
    request = YourService.Request()
    # Convert msg to request
    return request

translator = SpotServiceTranslator(
    service_name="/your/service",
    topic_name="/your/topic",
    service_type=YourService,
    msg_type=YourMsgType,
    request_builder=build_request,
    update_rate=0.1
)
```

See [spot_service_translator.py](spot_service_translator.py) for the base class and examples.
