# MicroROS-Scripts-with-STM32
Repository dedicated to facilitating the integration of microROS and STM32


## How to use

1. Open a STM32CubeIde Project on STM32CubeIde.

**Note: When the script is executed it will replace your ioc file, so be careful.**

2. Close .ioc file if it is opened.
3. Execute ./main.sh.
4. Choose your project.
5. Follow the instructions shown in the terminal.

[![Stm32CubeIde and MicroROS](https://img.youtube.com/vi/mPAE4mRnuys/0.jpg)](https://www.youtube.com/watch?v=mPAE4mRnuys)



## Files
| File                     | Description                                                                                                   |
|--------------------------|---------------------------------------------------------------------------------------------------------------|
| `main.sh`               | File to be executed.                                                                                          |
| `base_functions.sh`      | File with base functions, such as finding directories, checking for errors in functions, and others.          |
| `fixed_configurations.sh`| File with configurations you should not change.                                                              |
| `your_configuration.sh`  | File with configurations you can change.                                                                     |
| `microROS_functions.sh`  | File with functions related to microROS, such as cloning the correct repository, starting the agent, and others.|
| `stm32Cube_Functions.sh` | File with functions related to STM32, such as editing the `.ioc` file and the project properties.             |
| `style.sh`               | File with functions related to stylization.                                                                  |
| `extra_packages.repos`   | File with your extra packages for usage in micro-ROS. The control msgs come by default, with an added example package (`actuator_msgs`).|
