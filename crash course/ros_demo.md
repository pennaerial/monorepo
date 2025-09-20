Quick lil demo** !!!

By: David Zhan

---
### 🎯 Objective:

Introduce some preliminary ROS concepts by creating + running ROS2 nodes, modeling publisher/subscription model, visually see data being transmitted

---

## 🛠️ Prerequisites:

- OS: Ubuntu 22.04 or 20.04 (or WSL2 on Windows)
    
- ROS 2 Humble or Rolling installed
    
- `colcon` and `ros-dev-tools` installed
    
- Python 3.8+ (comes with ROS 2)
    
- Terminal and VSCode or any editor

ALTERNATIVE : since VM is down
https://robostack.github.io/GettingStarted.html

Follow this guide to get the ROS2 humble environment set up. 
Then, any ROS file can be run locally through the 


---

## 🧠 Concepts Covered:

- Basic ROS 2 Nodes
    
- Topics
    
- Messages
    
- Publisher/Subscriber in Python
    
- Using CLI to introspect the system
    

---

## 🚀 Activity: “Talker and Listener”

You’ll create two simple Python nodes:

- `talker`: Publishes random numbers
    
- `listener`: Subscribes to those numbers and prints them
    

---

## 📁 1. Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🧱 2. Create a Python Package

```bash

pixi shell -e humble # needed to set up env if not in virtual machine

cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_pubsub
```

ros2 pkg create --build-type ament_python py_pubsub
### 🧠 Command Overview

bash

```
ros2 pkg create --build-type ament_python py_pubsub
```

This creates a **Python-based ROS 2 package** named `py_pubsub`.

### 🔍 Breakdown of Each Part

- `ros2 pkg create` This is the base command to create a new ROS 2 package.
    
- `--build-type ament_python` Specifies the build system and language:
    
    - `ament_python` means the package will be built using **Python** and the **ament** build system (standard in ROS 2).
        
    - If you were using C++, you’d use `ament_cmake` instead.
        
- `py_pubsub` This is the **name of your new package**. It will create a folder called `py_pubsub` with the necessary files and structure.
    

### 📁 What Gets Created

After running this command, you’ll get a directory like this:

Code

```
py_pubsub/
├── package.xml           # Metadata about the package
├── setup.py              # Python setup script
├── setup.cfg             # Configuration for setuptools
├── resource/
│   └── py_pubsub         # Resource index file
├── py_pubsub/
│   └── __init__.py       # Python module (empty by default)
```

You can then start adding your **publisher and subscriber nodes** inside the `py_pubsub` Python module.


---

## 📝 3. Create the Talker Node

**File:** `py_pubsub/py_pubsub/talker.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'random_number', 10)
        self.timer = self.create_timer(1.0, self.publish_number)

    def publish_number(self):
        msg = String()
        msg.data = f'Random number: {random.randint(0, 100)}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
	main()
```

---

## 👂 4. Create the Listener Node

**File:** `py_pubsub/py_pubsub/listener.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String, 'random_number', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
	main()
```

---

## 🛠️ 5. Edit `setup.py`

Add these lines in `entry_points`:

This block tells setuptools/colcon to generate two terminal executables (`talker` and `listener`) that directly run your `main()` functions in the corresponding Python modules. This is the standard way ROS 2 Python nodes become runnable via `ros2 run`.

```python
entry_points={
    'console_scripts': [
        'talker = py_pubsub.talker:main',
        'listener = py_pubsub.listener:main',
    ],
},
```

---

## 🧱 6. Build and Source

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 🔧 What `colcon` Does

1. **Discovers Packages**
    
    - Looks in your workspace (`src/`) for ROS2 packages (with `package.xml` + build files).
        
2. **Builds Packages in the Right Order**
    
    - Automatically resolves dependencies.
        
    - E.g., if package B depends on A, `colcon build` will build A first.
        
3. **Supports Multiple Build Types**
    
    - Handles `ament_cmake` (C++), `ament_python` (Python), and others **in the same workspace**.
        
    - This is why we can mix Python and C++ nodes.
        
4. **Installs & Sets Up Environment**
    
    - Builds go into the `install/` directory.
        
    - After building, you run:
        
        `source install/setup.bash`
        
        so your shell knows about your packages, executables, and dependencies.
        
---

## 🚦 7. Run the nodes
Now that we set up the executables, we can run both nodes to see them interact with each other!! 

### In **Terminal 1**:

```bash
ros2 run py_pubsub talker
```

### In **Terminal 2**:

```bash
ros2 run py_pubsub listener
```

---

## 🔍 8. Inspect with ROS 2 CLI

Try these in another terminal:

```bash
ros2 topic list
ros2 topic echo /random_number
ros2 node list
ros2 node info /talker
```

---





