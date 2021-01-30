Franka Panda
============
Redis driver for the Franka Panda, modified for use with perls2. 

**NOTE:** The `master` branch was last tested on Ubuntu 16.04 on 11/10/2020.
**NOTE:** This repo is specifically intended for use with perls2, for the original Stanford IPRL Franka Panda Driver see https://github.com/stanford-iprl-lab/franka-panda-iprl

Installation
------------

This driver has been tested on Ubuntu 16.04 with C++14.

1. Set up your Panda robot and controller following the instructions on
   [this page](https://frankaemika.github.io/docs/getting_started.html).
   Connect your computer to the robot controller (not the robot directly).

2. CMake 3.11 or higher is required to download the external dependencies. Ubuntu
   16.04 comes with CMake 3.5. The easiest way to upgrade is through pip:

   ```
   pip install cmake
   ```

3. Install dependencies
   ```
   sudo apt install libpoco-dev
   ```

3. Build the driver

   ```
   cd <franka-panda.git>
   mkdir build
   cd build
   cmake ..
   make
   ```

4. If you run into an error during the cmake step, try installing the following:

   ```
   sudo apt install doxygen python3-dev
   ```

   Then clear the `build` folder and re-run cmake.

Driver Usage
------------

1. Open the robot interface by connecting to the ip address of the controller in
   your web browser (e.g. ```172.16.0.2```).

2. Open the User stop and the robot brakes through the web interface. (The robot lights should be Blue)

3. Launch a Redis server instance if one is not already running. 

   ```
   redis-server
   ```
   
4. Open a terminal and go to the driver's ```bin``` folder.

   ```
   cd <franka-panda.git>/bin
   ```

4. Launch the driver with a YAML configuration file.

   ```
   ./franka_redis_driver ../resources/default.yaml
   ```
   
5. Kill the driver with `<ctrl-c>`. This will reset Redis keys and terminate the
   driver threads gracefully. If the driver doesn't terminate, it may be stuck
   trying to connect to a non-existent gripper. To prevent this from happening,
   set `use_gripper` to `false` in the YAML configuration file.
   
Test the installation
---------------------
1. Follow the instructions below ("Driver Usage") to setup the driver. 

2. Open redis-cli in another terminal
   
3. Reset the robot to a neutral position using the redis-cli terminal. Make sure user stop is in hand.
```
set franka_panda::control::mode reset
```

4. Confirm the reset in the driver terminal window by pressing Enter

5. Set the robot to gravity compensation mode in the redis-cli terminal
```
set franka_panda::control::mode floating
```

6. The robot should now be in Gravity Compensation mode, and easily movable by hand.

7. Reset the robot as in step 3.

8. Set the robot to idle mode with the redis-cli-terminal:
```
set franka_panda::control::mode idle
```
9. Exit the driver with Ctrl + C. 

10. Engage the User stop so that the robot lights are now white. 

Important Notes
---------------
###  Control loop timing and tau_command_timeout
Because torque control requires a high frequency and consistent timing, the
following behavior will occur if the torque command key on redis remains stale
for a specified period of time:

1. If the torque command key ("franka::control::tau") remains the same for a period
of 10ms or greater, the driver will send zero torques ("floating" or "gravity 
compensation" mode). 

2. If the torque command key remains the same for a period greater than the
tau_command_timeout (specified in the driver's config file, default is 20ms), the
driver will exit with the message 'tau_command_timeout'. 

3. If the torque command key after being stale for 10-20ms, the driver will resume
sending torques.

This behavior is meant to address possible latencies that may occur with the redis-server 
or inconsistencies in the control loop. For the worst possible latencies, the robot will already
be floating before the driver exits. 

If this happens, simply flush the redis database and restart the driver. If this happens very
often, make sure no extra processes are running on the NUC. 

### Reset Behavior
When a robot reset is commanded, the driver will ask for confirmation from the user prior to executing the reset.
Simply press Enter on the driver terminal to initiate the reset. 

During reset commands the robot will use a joint space controller to return to a neutral position. 
This controller does not do any collision checking, so it is important that the user monitor the robot state and 
have the user-stop ready. 


Redis Keys
----------

The keys can be specified in the YAML configuration file (see
`<franka-panda.git>/resources/default.yaml`). For reference, the default keys
are:

### Robot control commands

The control mode should be set ***after*** the corresponding control command has
already been set (e.g. set `tau = "0 0 0 0 0 0 0"` before `mode = torque`), or
simultaneously with MSET. Otherwise, the robot may try to execute control with
stale command values.

- `franka_panda::control::tau`: Desired control torques used during torque
  control mode. \[7d array (e.g. `"0 0 0 0 0 0 0"`)\].
- `franka_panda::control::pose`: Desired transformation matrix from end-effector
  to world frame for `cartesian_pose` control, or desired delta pose for
  `delta_cartesian_pose` control. \[4x4 array (e.g.
  `"1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1"`)\].
- `franka_panda::control::mode`: Control mode. Note that the `cartesian_pose`
  controllers are blocking and must reach the target pose before receiving the
  next command. The mode will be set to `idle` after these controllers have
  finished running. \[One of {`"idle"`, `"floating"`, `"torque"`,
  `"cartesian_pose"`, `"delta_cartesian_pose"`}\].

### Gripper control commands

The gripper has two modes:
[grasp](https://frankaemika.github.io/libfranka/classfranka_1_1Gripper.html#a19b711cc7eb4cb560d1c52f0864fdc0d)
and
[move](https://frankaemika.github.io/libfranka/classfranka_1_1Gripper.html#a331720c9e26f23a5fa3de1e171b1a684).
Both of these commands are blocking and must finish before receiving the next
command.

The `move` command takes in a `width` and `speed`. The `grasp` command takes in
`width`, `speed`, `force`, and `grasp_tol`.

The control mode should be set ***after*** the control parameters have already
been set (e.g. set `width = 0` and `speed = 0.01` before `mode = move`), or
simultaneously with MSET.

- `franka_panda::gripper::control::width`: Desired gripper width. \[Positive double\].
- `franka_panda::gripper::control::speed`: Max gripper speed \[Positive double\].
- `franka_panda::gripper::control::force`: Max gripper force (used only for
  grasp c\ommand). \[Positive double\].
- `franka_panda::gripper::control::grasp_tol`: Width tolerances to determine
  whether object is grasped. \[Positive 2d array (e.g. `"0.05 0.05"`)\].
- `franka_panda::gripper::control::mode`: Gripper control mode. After a control
  command has finished, the driver will reset the mode to `idle`. \[One of
  {`"idle"`, `"grasp"`, `"move"`}\].

### Robot status

These keys will be set by the driver in response to control commands.

- `franka_panda::driver::status`: If the driver turns `off` (either due to a
  robot error or user interrupt signal), the controller should ***stop***
  immediately. Restarting the driver with an old controller already running is
  dangerous. \[One of {`"running"`, `"off"`}\].
- `franka_panda::control::status`: If the cartesian pose controller
  successfully reaches the target pose, the control status will be set to
  `finished`. This way the controller knows when to execute the next cartesian
  pose command. \[One of {`"running"`, `"finished"`, `"error"`}\].
- `franka_panda::gripper::status`: If the gripper is not running, the status
  will be `off`, and during a gripper command, it will be `grasping`. When a
  gripper command finishes, the `status` will be set to `grasped` if the object
  has been classified as grasped or `not_grasped` otherwise. \[One of {`"off"`,
  `"grasping"`, `"grasped"`, `"not_grasped"`}\].

### Robot sensor values

These keys will be set by the driver at 1000Hz.

- `franka_panda::sensor::q`: Joint configuration. \[7d array (e.g. `"0 0 0 0 0 0 0"`)\].
- `franka_panda::sensor::dq`: Joint velocity. \[7d array (e.g. `"0 0 0 0 0 0 0"`)\].
- `franka_panda::sensor::tau`: Joint torques. \[7d array (e.g. `"0 0 0 0 0 0 0"`)\].
- `franka_panda::sensor::dtau`: Joint torque derivatives. \[7d array (e.g. `"0 0 0 0 0 0 0"`)\].
- `franka_panda::gripper::sensor::width`: Gripper width. \[Positive double\].
