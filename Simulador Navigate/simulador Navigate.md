# Lume Autonomous Mobility System

The Lume Autonomous Mobility System has modules to assist in vehicle control and communication. It is a framework that operates with the IPC message exchange protocol and includes the following main modes:

- [*Simulator Mode (Navigate)*](../simulator_mode/simulator_mode.md)
- [*Playback*](../playback/playback.md)
- [*Record Log*](../record_log/record_log.md)
- [*Real World Use*](../real_world_use/real_world_use.md)

Portuguese version available [here](sistema_lume_mobilidade_autonoma_pt-BR.md) 🇵🇹.

*******
**Table of Contents**
- [Lume Autonomous Mobility System](#lume-autonomous-mobility-system)
  - [Astro System](#astro-system)
  - [The Process File](#the-process-file)
    - [Variables](#variables)
    - [Process Modules](#process-modules)
  - [Running a Process](#running-a-process)
    - [Solving an Error](#solving-an-error)
  - [Next Step](#next-step)

*******

## Astro System

**Astro** is a framework and has various modules located in the *src* folder, and when compiled, their binaries go to the *bin* folder.

<p align="center">
  <img height="250" src="./imgs/astro_dir.jpg"/>
</p>

The directory structure of the *bin* folder is such that each vehicle has its own folder containing its *.ini* file with its information and its *processes* for each mode, for example:

    .
    ├── atego1730_ufes_ype             # atego1730 folder that performed the arcelor route
    │   ├── astro-mercedes-atego1730-sensorbox-1.ini        # Vehicle's file
    │   ├── process-atego1730-navigate.ini                  # Navigate mode process
    │   ├── process-atego1730-playback-sensorbox-1.ini      # Playback mode process
    │   └── ...                                             # etc.
    └── ...

If you don't have Astro installed, follow the [Astro Installation Tutorial](../../../Installations/install_astro/README.md).

With Astro installed on the machine, run the `git pull` command inside the *astro/src* directory to download new system updates. By running `git status`, you can check which files have changes compared to those stored locally with the ones in the GitHub repository. If you are not familiar with these commands, [here's a GitHub guide](github_guia_simplificado.md).

Once synchronized with the GitHub version, also within the *astro/src* directory, run the `make clean` command to remove object files in the directory and `make` to compile with the new changes.

<div id='the-process-file'/>

## The Process File

A process is a file that ends with the *.ini* extension, organized into modules, and contains various programs that will be executed simultaneously by *proccontrol*. It is used to execute all modes of the Autonomous Mobility System, with the difference between each being the modules and programs present in each process.

The program called to execute the modules is *proccontrol*, which runs all programs on the command line specified by the passed process file.

Here's an example of a process:

<p align="center">
  <img width="1000" src="./imgs/process.jpg"/>
</p>

At the beginning of the file, the values of some variables are established, and then the file is organized by modules, each of which has a group of programs, with the *requested_state* column indicating whether it should be used or not.

### Variables
  - Line 1: Path to the map file
  - Line 2: Path to the graph file
  - Line 3: Path to the annotation file
  - Line 4: Server address
  - Line 5: Vehicle ID

The *command_line* column contains what should be written in the command line by *proccontrol*, which is the program executable and any arguments if needed.

### Process Modules
* Support Services: 
  - *./param_daemon*: Provides information about the vehicle being used and the surrounding area to other programs. It takes the vehicle file as an argument.
  
  - *./proccontrol_gui*: Generates the *proccontrol* interface showing the programs and their states.
  
  - *./playback*: Executes a log file and sends stored messages to other modules. It takes the log file as an argument.
  
  - *./playback_control*: Generates the *playback* control bar with start and pause buttons.
  
  - *./map_server*: Loads the vehicle position and the offline map from the map file information passed. It has the following arguments:
    -  *-map_path* <file_path>: path to the map file.
    -  *-map_x* <float_x>: global x-coordinate of the map.
    -  *-map_y* <float_y>: global y-coordinate of the map.
    -  *-block_map* \<state>: *on* or *off*, enables map generation with information within the specified directory.
    -  *-lanemap_incoming_message_type*: message type.
* Vehicle:
  -  *./simulator_ackerman*: Responsible for generating simulated data for the virtual vehicle.
  
  <br>
* Slam:
  -  *./localize_ackerman*: Uses sensor information from the baseServer to find the vehicle's position on the map provided by *param_daemon*. The *-mapping_mode* \<state>, *on* or *off*, indicates whether an offline map will be generated from the sensors.
  
  -  *./mapper*: Updates the offline map generated by the *map_server* module with real-time information (objects, people, etc). The *-map_path* <file_path> argument specifies the map file path.
  
  -  *./obstacle_distance_mapper*: Generates the obstacle distance map, cost map, and performs mobile object detection.
  
  -  *./fused_odometry*: Publishes instant odometry values (speed and steering angle) obtained by sensor fusion.  
  
  -  *./lidarodom*: Publishes estimated odometry from lidar readings, i.e., by comparing consecutive point clouds.

* Sensor Data Processing:
  - *./gps_xyz*: Vehicle GPS filter. Converts raw GPS data to system-used data, such as xyz position.
  
  - *./base_ackerman*: Applies (bias) and republishes corrected vehicle odometry.
  
  - *./neural_object_tracker*: Uses neural networks to detect pedestrians in the crosswalk.
  
  - *./multiple_object_tracker*: Uses neural networks to detect moving objects.
  
  - *./aruco_position_tracker*: Uses the ArUco library to detect marker (QR Code) positions in the world.
  
* Control Hierarchy:
  -  *./task_manager*: Sends a sequence of commands to the system contained in the mission file passed as a parameter. Commands in the file can include: defining Final Goal, defining graph and map, starting or interrupting autonomous operation, determining wait times between commands, etc. It has the following arguments:
     -   *-mission_from_user_app* \<state>, *on* or *off*, which determines whether the mission will be defined via the application.
     -   *-mission* <mission_path>, which passes the mission file.
     -   *-start_delay* \<state>, *on* or *off*, which determines whether a standard delay will be used before starting the mission.
  
  <br/>

  -  *./route_planner*: Accesses the graph file and from it publishes the desired position sequence that forms the path the vehicle should follow. It also publishes annotations relevant along this path. It has the argument *--graph* <graph_path> <annotation_path>, which are the graph and annotation files.
  
  -  *./offroad_planner*: Plans unstructured paths, i.e., paths that start or end outside a graph. Also responsible for reverse maneuver situations.

  -  *./behavior_selector*: Chooses the appropriate behavior in different situations detected by other modules.
  
  -  *./model_predictive_planner*: Returns vehicle acceleration based on the next desired position.
  
  -  *./rrt_path_follower*: Attempts to reduce latency in the actuation system.
  
  -  *./obstacle_avoider*: Controls vehicle acceleration to avoid obstacles.
  
* Interfaces:
  -  *./navigator_gui2*: Responsible for generating the graphical interface. It has the arguments *-map_path* <file_path> and *-annotation_path* <file_path>, which are the map and annotation files.
  
  -  *./viewer_3D*: Generates the 3D graph where the moving vehicle and sensor data are displayed.
  
  -  *./util_publish_initial_pose*: Indicates the initial vehicle position. It has arguments for x, y, and direction.

  - *./camera_viewer*: Generates the interface that displays the recorded vehicle camera during the journey.

  - *./traffic_light*: Traffic light detector.
  
* API:
  -  *./upstream_api*: Responsible for receiving messages from modules, subscribing, and sending the necessary messages to the WEB Monitoring System.
  
  -  *./downstream_api*: Responsible for receiving messages from the WEB Monitoring System and publishing the necessary messages to the rest of the modules.

## Running a Process

For a better understanding of the Lume Autonomous Mobility System, let's run a *Navigate Process*.

The first step is to run the `./central` command in the terminal within the *astro/bin* directory. The *central* program is responsible for establishing connection and allowing messages between modules.

If you attempt to run *central* while it is already running in another terminal, you'll receive the following error:

<p align="center">
  <img width="auto" src="./imgs/central_error.jpg"/>
</p>

In another terminal, in the same directory, we need to call *proccontrol* with the desired *process* as the argument. So, run the command `./proccontrol process-navigate-volta-da-ufes-sensorbox.ini`

### Solving an Error

Errors may occur when running a *process*. A common reason for this occurrence is when a file with its path is specified, but the file does not exist or has not been unpacked. For example, attempting to run the following *process*: `process-navigate-atego1730_ype-sensorbox-4.ini` that is inside the *atego1730_ype* car folder, this happened:

 <p align="center">
  <img width="auto" src="./imgs/process_dying.gif"/>
</p>

The first thing to do is click on the programs in yellow on the *Proccontrol GUI* and select "Stop Program," as they are dying and being called again by *proccontrol*, and then press Ctrl+C in the terminal to terminate.

To identify the program that was causing the problem, open the terminal and identify the program that was dying. Then, run it on the command line, and the error message will be displayed. In the case above, the error message was:

`PROCCONTROL (9891): ./mapper -map_path atego1730_ype/geodata/map_atego1730_ype_20230511-sensorbox4/map_files3/ (10033) exited UNCLEANLY (code = 255).`

While checking the path passed to *mapper*, it was found that the */map_files3* directory did not exist. To fix this error, you need to import the necessary map file for the program and place it in the specified directory.

## Next Step

Now that you understand how the **Lume Autonomous Mobility System** works and how to configure the **Astro System**, the next step is to understand the functioning of the Simulator Mode (Navigate). Follow [this tutorial](../simulator_mode/simulator_mode.md) to learn more.
