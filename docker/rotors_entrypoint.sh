#!/bin/bash

# Entrypoint for the rotors Docker container
function usage() {
    echo "Shell script ./`basename $0` - RotorS entrypoint for docker container"
    echo -e \
        "\t -h | --help \t Show help (this menu) and exit"

    echo "Exit codes of the script"
    echo -e \
        "\t 0 \t No problems encountered" "\n"\
        "\t 2 \t Invalid options given"
}

echo "Called: $0 $@"

# Parse all arguments
while (( $# )) ; do
    arg=$1  # Read argument
    shift   # Next argument for $@
    case "$arg" in
        "--help" | "-h")
            echo "Help options!"
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $arg (list: $arg $@)"
            # exit 2
            ;;
    esac
done

# Give bash access
bash
