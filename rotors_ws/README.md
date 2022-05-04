# RotorS workspace

This is the RotorS workspace which will be mounted directly to the Docker container (in the build process). It contains only the official RotorS repositories (as described in the setup).

> **Advice**: Best not to change anything here after setting up the Docker container. Do all the ROS-container-workspace experiments with the [custom_ws](./../custom_ws/) folder (which is _volume_ mounted at runtime). This folder is mounted during the build.

## Table of contents

- [RotorS workspace](#rotors-workspace)
    - [Table of contents](#table-of-contents)
    - [Setup](#setup)

## Setup

If you clone this repository using `--recursive`, the files will probably be there already. Otherwise, you can run the following commands in this folder (starting with only this README file).

Do the following to setup this workspace

1. Download the official [ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator) repository

    ```sh
    git clone git@github.com:ethz-asl/rotors_simulator.git src
    ```

    This is to download the main workspace in the src folder.
