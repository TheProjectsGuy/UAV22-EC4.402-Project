# Introduction to UAV Design - EC4.402 - Team Omicron

> **Note**: This is the docker branch, intended for Ubuntu 20.04 (LTS) and beyond systems. It will run the Rotors back-end (non-GUI) in a docker container and will use the GUI (front-end) and other components on the host system. For the main project, see the [main](https://github.com/TheProjectsGuy/UAV22-EC4.402-Project/tree/main) branch.

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
    - [References](#references)

## Contents

The contents of this folder are summarized below

| S. No. | Item Name | Description |
| :----- | :-------- | :---------- |
| 1 | [reading](./reading/README.md) folder | All reading material |
| 2 | [docker](./docker/) | Contains the Docker file (for containerizing the project) |

### Extra files

Some files that are for the project itself

| S. No. | Item Name | Description |
| :----- | :-------- | :---------- |
| 1 | [CONTRIBUTIONS.md](./CONTRIBUTIONS.md) | For contributors |
| 2 | [TODO.md](./TODO.md) | Task-list for the project |

## Setting up RotorS

Setting up the RotorS simulator

### Rotors Docker container

RotorS simulator natively works with Ubuntu Xenial (16.04 LTS). We can use Docker to virtualize it.
There are better alternatives like [shrmpy/rotors](https://github.com/shrmpy/rotors) available. However, this repository also comes with a Docker solution.

1. To build the docker container, run the following

    ```bash
    # In the root folder of the repository
    docker build --tag rotors:latest -f ./docker/Dockerfile .
    ```

- The `rotors_src` was created by running the following commands (in the root project folder)

    ```bash
    git clone git@github.com:ethz-asl/rotors_simulator.git rotorsws_src
    ```

- Run the container using

    ```bash
    docker run --network=host -i -t --rm --name "rotors-xenial" rotors:latest bash
    ```

## References

- Rotors Simulator: [GitHub](https://github.com/ethz-asl/rotors_simulator), [Paper](https://link.springer.com/chapter/10.1007/978-3-319-26054-9_23), [ROS Wiki](https://wiki.ros.org/rotors_simulator)
