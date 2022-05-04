# Introduction to UAV Design - EC4.402 - Team Omicron

**Team Name**: Omicron <br>
**Project Topic**: Aerial Photography and Videography of sites <br>
**Team Members**

| S. No. | Name | Roll No. | EMail-ID |
| :---- | :---- | :------- | :------- |
| 1 | Avneesh Mishra | 2021701032 | avneesh.mishra@research.iiit.ac.in |
| 2 | M Aditya Sharma | 2021701046 | aditya.sharm@research.iiit.ac.in |

> **Note**: The docker version is available on the [docker](https://github.com/TheProjectsGuy/UAV22-EC4.402-Project/tree/docker) branch.

## Table of contents

- [Introduction to UAV Design - EC4.402 - Team Omicron](#introduction-to-uav-design---ec4402---team-omicron)
    - [Table of contents](#table-of-contents)
    - [Contents](#contents)
        - [Extra files](#extra-files)
    - [Setting up RotorS](#setting-up-rotors)
        - [ROS Neotic - Local installation](#ros-neotic---local-installation)
    - [References](#references)

## Contents

The contents of this folder are summarized below

| S. No. | Item Name | Description |
| :----- | :-------- | :---------- |
| 1 | [reading](./reading/README.md) folder | All reading material |

### Extra files

Some files that are for the project itself

| S. No. | Item Name | Description |
| :----- | :-------- | :---------- |
| 1 | [CONTRIBUTIONS.md](./CONTRIBUTIONS.md) | For contributors |
| 2 | [TODO.md](./TODO.md) | Task-list for the project |

## Setting up RotorS

Setting up the RotorS simulator

### ROS Neotic - Local installation

If you have ROS Noetic (tested on WSL - Ubuntu 20.04), a local installation would involve the following steps (assuming that ROS is already installed and sourced).

1. Create a workspace for the `rotors_simulator` stuff

    ```bash
    mkdir ~/rotors_simulator
    cd ~/rotors_simulator
    ```

    This will be the main workspace. It could be any directory really. Use this **only** for the official RotorS installation, so it's a good idea to not have anything else sourced when setting this up (we want this to be independent). Verify this using

    ```bash
    echo $ROS_PACKAGE_PATH | sed 's/:/\n/g'
    ```

    You should see only the `/opt/ros/noetic/share` here. If you don't, you're sourcing some workspaces!

2. Clone the official [ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator) repository in the workspace folder. This workspace will contain everything 'rotors' related.

    ```bash
    git clone git@github.com:ethz-asl/rotors_simulator.git src
    ```

    This will clone all code (it will take some time).

3. Install some dependencies for ROS. The RotorS simulator is build on top of other dependencies (they've listed them out).

    Install [octomap](https://wiki.ros.org/octomap): A map representation using voxels.

    ```bash
    sudo apt install ros-noetic-octomap-ros \
        ros-noetic-octomap-msgs ros-noetic-octomap
    ```

    Install [mavros](http://wiki.ros.org/mavros): MAVLink communication and proxy for Ground Control Station.

    ```bash
    sudo apt install ros-noetic-mavros
    ```

4. Download dependencies using `wstools`

    Install [wstools]: Web services for python. Try [this](http://wiki.ros.org/wstool) if the command below doesn't work.

    ```bash
    sudo apt install python3-wstool
    ```

    After that, initialize and update the wstool

    ```bash
    # Initialize the workspace
    wstool init
    # Add the .rosinstall file
    wstool merge ./rotors_hil.rosinstall
    # Update the file
    wstool update
    ```

    This will create two folders
    - `mav_comm` as in [ethz-asl/mav_comm](https://github.com/ethz-asl/mav_comm): ETH Zurich has their own message and service definitions for MAV communication. These have to be built from source (as they're not available in apt store).
    - `mavlink` as in [mavlink/mavlink-gbp-release](https://github.com/mavlink/mavlink-gbp-release/tree/upstream): MAV Message Marshalling (serialization) Library that handles messaging protocols.

    Whenever you need to **update**, simply run

    ```bash
    wstool update
    ```

5. Install `future` for python backward compatibility

    ```bash
    # If you don't have pip3 already!
    sudo apt install python3-pip
    # Install future
    pip3 install future
    ```

    You may have to add `$HOME/.local/bin` to `PATH`. Do this by adding the following to the `~/.bashrc` or the `~/.zshrc` file

    ```bash
    export PATH=$PATH:$HOME/.local/bin
    ```

    Now, the path should be visible in the following command

    ```bash
    # Show each path on a newline
    echo $PATH | sed 's/:/\n/g'
    ```

6. Install `glog`: `rotors_gazebo_plugins` requires `glog` for logging

    ```bash
    sudo apt install libgoogle-glog-dev
    ```

7. Build the package

    Run the following to build everything in the package

    ```bash
    catkin_make_isolated
    ```

    Make sure you run this in the ROS Workspace (the `~/rotors_simulator` folder in step 1). If you ever get the build messed up (say by sourcing another workspace when building this one), you can do this all over again by running

    ```bash
    # Clean the existing build
    rm -rf ./build ./build_isolated ./devel ./devel_isolated
    # Build again
    catkin_make_isolated
    ```

8. Make sure you source this workspace in the end (after building everything)

    Add this to `~/.bashrc` (use `.zsh` instead for `.bash` for zsh shell)

    ```bash
    source $HOME/rotors_simulator/devel/setup.bash
    ```

    After that, source the file and the following command should show the workspace in the ROS path

    ```bash
    echo $ROS_PACKAGE_PATH | sed 's/:/\n/g'
    ```

## References

- Rotors Simulator: [GitHub](https://github.com/ethz-asl/rotors_simulator), [Paper](https://link.springer.com/chapter/10.1007/978-3-319-26054-9_23), [ROS Wiki](https://wiki.ros.org/rotors_simulator)
