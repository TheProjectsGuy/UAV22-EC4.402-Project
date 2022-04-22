# Introduction to UAV Design - EC4.402 - Team Omicron

**Team Name**: Omicron <br>
**Project Topic**: Aerial Photography and Videography of sites <br>
**Team Members**

| S. No. | Name | Roll No. | EMail-ID |
| :---- | :---- | :------- | :------- |
| 1 | Avneesh Mishra | 2021701032 | avneesh.mishra@research.iiit.ac.in |
| 2 | M Aditya Sharma | 2021701046 | aditya.sharm@research.iiit.ac.in |

## Table of contents

- [Introduction to UAV Design - EC4.402 - Team Omicron](#introduction-to-uav-design---ec4402---team-omicron)
    - [Table of contents](#table-of-contents)
    - [Contents](#contents)
        - [Extra files](#extra-files)
    - [Setting up RotorS](#setting-up-rotors)
        - [Rotors Docker container](#rotors-docker-container)
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

### Rotors Docker container

RotorS simulator natively works with Ubuntu Xenial (16.04 LTS). We can use Docker to simulate it.
There are better alternatives like [shrmpy/rotors](https://github.com/shrmpy/rotors) available. However, this repository also comes with a Docker solution.

1. To build the docker container, run the following

    ```bash
    cd docker
    docker build . -t rotors:xenial
    ```

### ROS Neotic - Local installation

If you have ROS Noetic (tested on WSL), a local installation would involve the following steps (assuming that ROS is already sourced)

1. Create a workspace for the `rotors_simulator` stuff

    ```bash
    mkdir ~/rotors_simulator
    cd ~/rotors_simulator
    ```

    This will be the main workspace

2. Clone the official [ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator) repository in the workspace folder. This workspace will contain everything 'rotors' related.

    ```bash
    git clone git@github.com:ethz-asl/rotors_simulator.git src
    ```

    This will clone all code (it will take some time).

3. Install some dependencies for ROS. The RotorS simulator is build on top of other dependencies (they've listed them out).

    Install [octomap](https://wiki.ros.org/octomap): A map representation using voxels.

    ```bash
    sudo apt install ros-noetic-octomap ros-noetic-octomap-msgs -y
    ```

## References

- Rotors Simulator: [GitHub](https://github.com/ethz-asl/rotors_simulator), [Paper](https://link.springer.com/chapter/10.1007/978-3-319-26054-9_23), [ROS Wiki](https://wiki.ros.org/rotors_simulator)
