# Model Tools

Contains all details for UAV models

## Table of contents

- [Model Tools](#model-tools)
    - [Table of contents](#table-of-contents)
    - [Launch commands](#launch-commands)
    - [Problems](#problems)
        - [Model not showing](#model-not-showing)

## Launch commands

To launch and inspect a XACRO model from a package, use the following

```bash
roslaunch model_tools inspect_mav.launch mav_name:=pelican
```

## Problems

### Model not showing

Some `.dae` files could be corrupted or the format might not be compatible with RViZ. Use primitive shapes in that case.
