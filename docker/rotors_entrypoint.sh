#!/bin/bash

# Entrypoint for the rotors Docker container
function usage() {
    # Heading
    echo "Script ./`basename $0` - Entrypoint of RotorS container"
    # About the script
    echo -e "The container sets up everything needed to run RotorS" "\n"\
            "This script currently does the following" "\n"\
            "1. Sources the ROS Kinetic installation (if no -n)" "\n"\
            "2. Builds the RotorS workspace (if '-r' is passed)" "\n"\
            "3. Sources the RotorS workspace (if no -t)" "\n"\
    echo;
    # Help options
    echo "Options:"
    echo -e \
        "\t -h | --help \t Show help (this menu) and exit" "\n"\
        "\t -n | --no-ros \t Do not source ROS Kinetic" "\n"\
        "\t -r | --build-rotors \t Build RotorS ROS Workspace" "\n"\
        "\t -t | --no-rotors \t Do not source the RotorS workspace \n"\
        ""
    echo;
    # Exit codes
    echo "Exit codes:"
    echo -e \
        "\t 0 \t No problems encountered" "\n"\
        "\t 2 \t Invalid options given"
}

echo "Called: $0 $@"

# Variables (settings)
rosws_source=true
rotorsws_build=false    # Don't build by default
rotorsws_source=true

# Parse all arguments
while (( $# )) ; do
    arg=$1  # Read argument
    shift   # Next argument for $@
    case "$arg" in
        # Help options!
        "--help" | "-h")
            usage
            exit 0
            ;;
        # ROS workspace
        "-n" | "--no-ros")
            rosws_source=false
            ;;
        # RotorS workspace (build)
        "-r" | "--build-rotors")
            rotorsws_build=true
            ;;
        # RotorS workspace (source)
        "-t" | "--no-rotors")
            rotorsws_source=false
            ;;
        *)
            echo "Unknown option: $arg (list: $arg $@)"
            exit 2
            ;;
    esac
done

# set -x
# set -n

# Source ROS workspace
if $rosws_source; then
    echo "Sourcing ROS Kinetic"
    source /opt/ros/kinetic/setup.bash
else
    echo "Not sourcing ROS Kinetic"
fi

# Build RotorS workspace
rotorsws_path="/ros_workspaces/rotors_ws"
if $rotorsws_build; then
    echo "Building the RotorS workspace"
    old_pwd = $PWD
    cd $rotorsws_path
    catkin build
    echo "RotorS workspace successfully built"
    cd $old_pwd
else
    echo "Not building the RotorS workspace"
fi

# Source RotorS workspace
if $rotorsws_source; then
    echo "Sourcing the RotorS workspace"
    if [ -f "$rotorsws_path/devel/setup.bash" ]; then
        # Found file, source it
        source "$rotorsws_path/devel/setup.bash"
        echo "Sourced the RotorS workspace"
    else
        echo "File not found - $rotorsws_path/devel/setup.bash"
        echo "You may have to build the RotorS workspace first! (see -h)"
    fi
else
    echo "Not sourcing the RotorS workspace"
fi

# Setup completed
echo "---------- Setup completed ----------"
# Show workspace paths
if [[ $ROS_PACKAGE_PATH ]]; then
    echo "ROS Workspaces are"
    echo $ROS_PACKAGE_PATH | sed s/:/\\n/g | awk '{print("  - " $1)}'
else
    echo "Variable ROS_PACKAGE_PATH not found"
fi

# Give bash access
echo -e "Starting bash session\n\n"
bash
