# VEX Robotics- HKIS 936A Codebase

Welcome to the HKIS Robotics Club's codebase for VEX Robotics using the VEX V5 system.

## Contribution

Main author: Jiahong (Ericsson) Lin- `ericssonl07` (HKIS Class of 2025)
Contributors:
- Qi (Aaron) You- `coon-hound` (HKIS Class of 2025)
- Qinan (Andy) Liu- `PhantomKing99999` (HKIS Class of 2026)
- Lucas Tong- `LuTong00` (HKIS Class of 2026)
- Yushin Chang- `orangeskyy` (HKIS Class of 2028)
- Jiaying (Niu Niu) Fan- `purpleskyeee` (HKIS Class of 2028)

## Overview and File Structure

This repository contains code to manipulate the robot's movement during the control and autonomous period. The code is dependency-free, except for the C++ Standard Library. All code (except the `tests` directory) is separated into header files (in the `include` folder, as `.hpp` files) and implementation files (in the `src` folder, as `.cpp` files). The project is structured into several key components:

- **Autonomous Control**: The `autonomous` directory contains code for various autonomous routines, including odometry, path following, and pure pursuit algorithms. The `Odometry` class tracks the robot's position and rotation, while the `PurePursuit` class helps the robot follow a predefined path. In addition, a helper `AbstractMotorGroup` class is defined as a wrapper around VEX's `vex::motor_group` class to add some extra functionality, and a helper `PID` class is defined to simplify PID controller operations.

- **Pathing**: The `pathing` directory includes implementations for path generation and interpolation. The `CubicSpline` class provides cubic spline interpolation to create smooth paths between waypoints. The `Matrix` class provides a basic matrix implementation to allow for solving systems of equations. The `Path` class provides functionality to make path calculation simpler.

- **Bot**: The `BotBase` class contains the main interface for controlling the robot. More information in the section below, `Usage (Functions)`.

- **Testing**: The `tests` directory contains various test functions to validate the functionality of different components, such as `test_spline` for spline interpolation and `test_purePursuit` for path following.

- **Main Program**: The `main.cpp` file initializes the robot and starts the control and autonomous tasks. It ensures that the odometry is zeroed and keeps the threads alive during operation. It currently implements code for the VEX High Stakes game (2024/2025).

- **Configuration**: The `config.hpp` file contains configuration settings for the robot, such as motor ports and sensor configurations. This structure ensures modularity and ease of maintenance, allowing for efficient development and testing of new features. Additionally, conditional compilation allows testing of code without a robot (utilizing Odometry to track a virtual robot's movement using only a VEX brain and two motors). Alternatively, conditional compilation is used to quickly switch between code for a VEX Field Controller (competition mode) and regular testing.

## Usage (Functions)

More documentation is available for each function with comments (`/* ... */`); programmers using Visual Studio Code with C++ Intellisense installed will be able to hover to view documentation.

1. Constructor

- Function signature: `BotBase(AbstractMotorGroup* left, AbstractMotorGroup* right, double base_width, double wheel_radius, double pursuit_distance, double gear_multiplier, double initial_x = 0.0, double initial_y = 0.0, double initial_rotation = 0.0, int thread_sleep = 10);`
- Description: This function constructs the base with specified motor groups, base dimensions, pursuit parameters, and initial pose. The last parameter, `thread_sleep`, adjusts the frequency of odometry update in milliseconds.

2. Path following

- Function signature 1: `void follow_path(std::vector<double> x, std::vector<double> y, double tolerance, double speed_factor = 1.0, int num_points = 50);`
- Function signature 2: `void follow_path(Path path, double tolerance = 5.0, double speed_factor = 1.0);`
- Description: This function follows the specified path with a given tolerance and speed factor. The path is either a `Path` object or two `std::vector<double>` objects specifying (separately) the $x$ and $y$ coordinates. The tolerance is given in centimeters, and the speed factor simply slows the speed to allow for more accurate motion.

3. Turning (turn to)

- Function signature: `void turn_to(double angle, double tolerance = 0.01, double speed_factor = 1.0);`
- Description: This function turns the robot to a set angle in radians. It is smart enough to account for coterminal angles, so a robot at 359 degrees turning to 1 degree will merely turn 2 degrees clockwise, not 358 degrees counterclockwise. Note that angle measures follow the mathematical convention where 0 is positive-x and positive angles indicate counterclockwise turns.

4. Turning (turn by)

- Function signature: `void turn(double angle, double tolerance = 0.01, double speed_factor = 1.0);`
- Description: This function turns the robot by a set angle in radians. Similar conventions and functionality applies as the `turn_to` function.

5. Forward

- Function signature: `void forward(double distance, double tolerance = 5.0, double speed_factor = 1.0);`
- Description: This function moves the robot forward by a specified distance. It is internally implemented by calculating a `Path` object with only two points.

## Experimental Features

- Keybinding (controller code): using a set of predefined "controller command combinations" to perform pre-set tasks, such as navigating to a certain point on the field. The intended implementation uses `std::map<std::vector<vex::button>, std::function<void()>>` to map key combinations to a lambda, function, or closure; however, VEX currently does not seem to support this function. May implement a wrapper `button` class that supports `std::map`.

## Troubleshooting

Common sources of error:
- Are the motors properly wired? This may be indicated by drift
- Are the ports correct? This may be indicated by other odd behavior
- Are the reversals correct? This may be indicated by one side seizing up
- Is the `MODE` macro in `config.hpp` correct? This will affect motor setup due to the conditional compilation.
- Are any paths traced closed, or near-closed (start and end are close together)? This will lead to no movement being executed. If it is the case, consider separating the paths into two or rerouting.

## Reference (Mathematics)

To briefly summarize the mathematical components of the code:
- Odometry uses motor encoder readings and applies calculations to return differential changes in heading and position, which are integrated (added) together over time.
- Pathing takes a set of input points $(x_{i}, y_{i})$ and parametrizes them into two sets of inputs $(t_{i}, x_{i})$ and $(t_{i}, y_{i})$ since most input paths are not functions. Then, for $n+1$ input points, the program sets up and solves a system of $n$ piecewise cubics with $4n$ variables satisfying a certain set of continuity and smoothness restraints which return $4n$ equations. Then the path can be interpolated by calculating the value of the piecewise function at any given $t$ within the range of the path.

For a detailed and formalized statement of the above concepts, visit the full document of derivations in the file `Mathematics in Autonomous Control- VEX Robotics.pdf`.

## GitHub Setup (936A Reference)

First-time setup:
1. Install Visual Studio Code
2. Install the VEX Extension in Visual Studio Code.
3. Under the VEX Extension, select "New Project" and create a blank C++ project (without spaces). VEX will automatically configure makefiles, the `vex` folder, and the `.vscode` properties.
4. In MacOS Terminal, navigate to `~/Documents/vex-vscode-projects/` by entering `cd ~/Documents/vex-vscode-projects/<insert project name here>`.
5. Set up a new git repository and link to remote by entering `git init` and `git remote add origin <insert link to this repository here>`.
6. Remove conflicting files bt entering `rm .gitignore`, `rm ./src/main.cpp`, and `git add .` 
7. Pull the code by entering `git pull origin <insert branch name here>`. The default branch is called `main`.

Pushing code:
1. Ensure that you are on the correct directory with `cd ~/Documents/vex-vscode-projects/<insert project name here>`.
2. Ensure that you are developing on the correct branch (only push to the main branch if a feature is fully tested and working, or if you are the lead developer). To switch to a branch, type `git checkout <insert branch name here>`. To create a new branch, type `git checkout -b <insert branch name here>`.
3. Stage files for a commit with the command `git add <insert filename(s) here>`. For files in folders, use the relative path.
4. Create the commit with the command `git commit -m "<insert commit message here>"`.
5. Push the commit with the command `git push origin <insert branch name here>`.

Merging and Pull Requests:
When features on development are ready to be merged into the main branch, submit a pull request. A lead developer will review the code and resolve merge conflicts. Once the code is merged, you may delete the development branch with the command `git branch -d <insert branch name here>`.

For more practice with Git, refer to the visualization website at https://git-school.github.io/visualizing-git/.
