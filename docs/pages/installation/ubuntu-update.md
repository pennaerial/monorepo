## Updating from `22.04`
First update the system
```bash
sudo apt update && sudo apt upgrade
```

Remove `ros2` humble:
```bash
sudo apt remove '~nros-humble-*' && sudo apt autoremove

sudo apt remove ros2-apt-source
sudo apt update
sudo apt autoremove
sudo apt upgrade # Consider upgrading for packages previously shadowed.
```

Upgrade to Ubuntu `24.04`:
```bash
sudo do-release-upgrade
```
## Installing `ros2` Jazzy
Follow the instructions [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
## Installing PX4-Autopilot
After completing the upgrade to `24.04`, remove the old version of `protobuf` and `protoc` that might mess up with PX4 `1.17`:
```bash
sudo apt-get remove --purge protobuf-compiler libprotobuf-dev
sudo apt-get autoremove
sudo apt-get update
sudo apt-get install protobuf-compiler libprotobuf-dev pkg-config
```

For reinstalling PX4, the easiest solution is just to remove the current PX4-Autopilot directory and do the [installation instructions](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu) from scratch. For step 1, make sure the branch is specified as `-b release/1.17`. **After rebooting**, if you source ros2, you might get an error related to argcomplete. To fix this, run:
```bash
rm ./.local/bin/register-python-argcomplete
```
Also make sure you have argcomplete installed via apt:
```bash
sudo apt install python3-argcomplete
```
Check if PX4 works after:
```bash
make px4_sitl
make px4_sitl gz_x500
```
## Monorepo
To repair `ros2` dependencies, run:
```bash
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update
rosdep install -r --from-paths src -i -y --rosdistro jazzy

rosdep install --from-paths src -y --ignore-src # try if the top command doesn't work
```
`jazzy` uses a different release of our middleware, so we need to reinstall it as well. The instructions are on the PX4 `ros2` website, but it's here for reference (**make sure you remove your old one**):
```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

`jazzy` now supports binary installations for `ros-gz`, so we've removed the dependency from `colcon build` to dramatically speed up compile times. To install ([source](https://github.com/gazebosim/ros_gz/tree/jazzy)):
```bash
# Add https://packages.ros.org
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update

# install ros_gz
sudo apt install ros-jazzy-ros-gz
```
Make sure you source or restart your terminal before rebuilding
## Troubleshooting
If you get an error asking you to install dependencies for harmonic, try appending `GZ_VERSION=jetty` and removing the build files via `rm -rf build` in the `PX4-Autopilot` directory, then rebuilding via the `make` command

If you get an issue with middleware launching (something like `MicroXRCEAgent: error while loading shared libraries: libspdlog.so.1`), update MicroXRCEAgent to `2.4.3` via the instructions [here](https://docs.px4.io/main/en/ros2/user_guide#setup-micro-xrce-dds-agent-client). 
## References
- [PX4 Setup](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu)
- [Gazebo Jetty](https://gazebosim.org/docs/jetty/install_ubuntu/)
