# Guidance of Running the Single Drone Payload Estimation Project

## Step of Running the Project: 

1. Install ROS Melodic, Gazebo 9 and QGroundControl.

2. Pull PX4-Autopilot from this page.

3. Run `cd $path/PX4-Autopilot`, then run the command `make px4_sitl_default gazebo`.

4. Write the following lines at the bottom of ~/.bashrc:
* `source Tools/setup_gazebo.bash $path/PX4-Autopilot $path/PX4-Autopilot/build/px4_sitl_default`
* `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:{$path}/PX4-Autopilot`
* `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:{$path}/PX4-Autopilot/Tools/sitl_gazebo`
* `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{$path}/PX4-Autopilot/fsc_models`

5. Run `source ~/.bashrc`.

6. Create airframes named `single_iris_payload`, `single_iris_payload_visual` and `single_iris_payload_visual_aruco` in `~/.ros/etc/init.d-posix/airframes` respectively by entering the folder and running the folloiwing commands:
`cp 10016_iris 3014_single_iris_payload`: model with payload
`cp 10016_iris 3015_single_iris_payload_visual`: model with payload and camera
`cp 10016_iris 3016_single_iris_payload_visual_aruco`: model with camera and payload with identifiers

7. Apply the same airframe creating processes (step 8) in the following folders:
* `~/.ros/sitl_iris_0/etc/init.d-posix/airframes`
* `~/.ros/sitl_iris_1/etc/init.d-posix/airframes`
* `~/.ros/sitl_iris_2/etc/init.d-posix/airframes`

8. Run one of the following commands to launch the program: 
`roslaunch px4 single_iris_payload.launch` for model with payload,
`roslaunch px4 single_iris_payload_visual.launch` for model with payload and camera,
`roslaunch px4 single_iris_payload_visual_aruco.launch` for model with camera and payload with identifiers,
Those launch files will launch mavros and start the rosbag writing process simultaneously.
To launch a windly world, use `world_name:=windy`; 
To write rosbag, use `write_rosbag:=true` (aruco case only)

9. Launch QGroundControl, click `Plan` at the upper-left corner, then click `File` -> `Storage` -> `Open` to load the QGroundControl path file (.plan file). Then, click `Upload` at the top of the window. After that, click `Fly` at the upper-left control bar to return to the main page.

10. To launch the drone, when there is a sliding component presenting at the bottom showing `Takeoff from ground and start the current mission`, slide to take-off. If not, go back to `Plan` and click `Upload` at the top of the window again and return to `Fly`.

11. After launching, the ground truth of the pose of the payload is stored in the topic: `/gazebo_ground_truth_payload`.

## Parameter modifications:
* Location of rosbag saving: `rosbag_writer.py`, Line 48: `self.bag_loc`
* Weight of payload: `PX4-Autopilot/fsc_models/single_iris_payload_visual_aruco.sdf`, Line 691, `<mass>${weight}</mass>`
* Wind speed: `PX4-Autopilot/fsc_worlds/windy.world`, Line 20, `<windVelocityMean>${wind_speed}</windVelocityMean>`

## To generate new marker:
1. Go to `scripts/aruco_combine.py`: 

2. At Line 151, specify the absolute path that the generated marker image will store in the format: `.../final.png` 

3. At Line 152, the three parameters of create_dae() are 
* the absolute path to the proto .dae structure file (dae_struct.dae in the drive); 
* the absolute path to the generated .dae file; 
* the generated image, should be the same as Line 150. 
 
4. Then, run `python aruco_combine.py`

5. After finishing running the scripts, move the generated .dae file (second parameter of Line 151) to `PX4-Autopilot/fsc_models/${model name}/meshes`

6. Also, move the generated image (parameter of Line 150) to `PX4-Autopilot/fsc_models/${model name}/materials/textures`

In our case, ${model name} is single_iris_visual_QR_code_expanded, and it is recommended to set the final destinations (.../meshes and .../textures) of the .dae and .png files as the corresponding parameters in Line 150 and 151. 

## Location of files in drive:
Path to the Drive: https://drive.google.com/drive/folders/1bcr7QfcJM_zpQUhwNJ2592s2CpKBPATb

## Simulation Environment:
* Ubuntu 18.04
* ROS Melodic
* PX4 v1.12.1
* Gazebo 9
* MAVROS v1.8.0
* MAVLink v2021.3.3
* QGroundControl v4.1.3
* OpenCV v3.2.0
