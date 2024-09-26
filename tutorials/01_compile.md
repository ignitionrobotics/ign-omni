# How to compile it

## Install Gazebo

```bash
sudo apt update
sudo apt install python3-pip wget lsb-release gnupg curl
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-vcstool python3-colcon-common-extensions
sudo apt-get install git libfreeimage-dev

sudo apt-get install gz-harmonic
```

For more information, see https://gazebosim.org/docs/harmonic/install_ubuntu_src.

# Compile gz-omni

We need to compile some Gazebo packages from source with a specific flag due the `omni-client` library.
To make this process simple we have created the [`gz-omni-meta` repository](https://github.com/gazebosim/gz-omni-meta).

To compile this libraries you should run:

```bash
mkdir -p ~/gz-omni/src
cd ~/gz-omni/src
git clone https://github.com/gazebosim/gz-omni-meta
vcs import . < gz-omni-meta/repos.yaml
cd protobuf
git -C . apply ../gz-omni-meta/protobuf-cmake.patch
cd ~/gz-omni
colcon build --merge-install --event-handlers console_direct+ --packages-select protobuf
cp src/gz-omni-meta/colcon.meta .
colcon build --merge-install --event-handlers console_direct+ --packages-up-to gazebo-omniverse1
```

You can ignore the following message:

```bash
WARNING:colcon.colcon_cmake.task.cmake.build:Could not run installation step for package 'gazebo-omniverse1' because it has no 'install' target
```

**Note: There will be 2 builds of gazebo, the default build when gazebo-harmonic is compiled from source, and a special build with pre cxx11 abi compiled as part of gazebo-omniverse.**
