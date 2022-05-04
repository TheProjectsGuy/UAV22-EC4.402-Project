# RotorS workspace

This is the RotorS workspace which will be mounted directly to the Docker container (in the build process). It contains only the official RotorS repositories (as described in the setup).

> **Advice**: Best not to change anything here after setting up the Docker container. Do all the ROS-container-workspace experiments with the [custom_ws](./../custom_ws/) folder (which is _volume_ mounted at runtime). This folder is mounted during the build.
> **Strong Advice**: Do not build this package outside the docker container.

## Table of contents

- [RotorS workspace](#rotors-workspace)
    - [Table of contents](#table-of-contents)
    - [Setup](#setup)
        - [Optional: Build only once](#optional-build-only-once)

## Setup

If you clone this repository using `--recursive`, the files will probably be there already. You can directly go to the [optional step](#optional-build-only-once). Otherwise, you can run the following commands in this folder (starting with only this README file) to setup everything from scratch.

Do the following to setup this workspace (`pwd` in this folder)

1. Download the official [ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator) repository

    ```sh
    # The official repository
    git clone git@github.com:ethz-asl/rotors_simulator.git src
    # My fork (you won't have to do step 2)
    git clone --recursive git@github.com:TheProjectsGuy/rotors_simulator.git src
    ```

    This is to download the packages in the `src` of the workspace mount (this folder in docker container).

2. Download the (project) dependencies from GitHub (if you used the official repository in step `1.`)

    Run the following in the `src` folder (all packages here).

    Download the `mav_comm` repository from [ethz-asl/mav_comm](https://github.com/ethz-asl/mav_comm)

    ```sh
    git clone git@github.com:ethz-asl/mav_comm.git ./mav_comm
    ```

### Optional: Build only once

- Instead of having to build the RotorS workspace every time you `docker run`, you could have the build files located in this folder (same level as `src`) itself. This is optional will save a few minutes in the beginning of each run. The build files are not included when you clone this repository.

    Build the Docker container (run this in the repository root)

    ```sh
    docker build --tag rotors:latest -f ./docker/Dockerfile .
    ```

    Run and mount this directory as a separate volume (overriding the COPY command's mount). Do not build the `RotorS` workspace (we'll do it manually for this mount) and do not source the rotors workspace (pass `-t` in the end).

    ```sh
    # Check for name conflicts
    docker ps -a
    # Run the image in a container
    docker run --network=host -i -t --rm --name "rotors-xenial" -v $PWD/rotors_ws:/ros_workspaces/rotors_ws rotors:latest -t
    ```

    Inside the container, we will run the following

    ```sh
    # Go to the workspace mount (volume)
    cd /ros_workspaces/rotors_ws
    # Build the workspace
    catkin build
    ```

    After this, you should see the `devel`, `build` and `logs` folder. You can now exit the container. The folders should still be visible in this `rotors_ws` folder.

    After this, you can build the docker image again (the folder `rotors_ws` has changed) using the following command (run the root folder of the project)

    ```sh
    docker build --tag rotors:latest -f ./docker/Dockerfile .
    ```
