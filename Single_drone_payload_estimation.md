# Guidance of Running the Single Drone Payload Estimation Project

## Step of Running the Project: 

(Steps with * are only needed to be conducted if pulling PX4-Autopilot from FSC-Labs)

1. Install ROS Melodic, Gazebo 9 and QGroundControl.

2. Pull PX4-Autopilot from this page or FSC-Labs.

3. Run `cd $path/PX4-Autopilot`, then run the command `make px4_sitl_default gazebo`.

4. Write the following lines at the bottom of ~/.bashrc:
* `source Tools/setup_gazebo.bash $path/PX4-Autopilot $path/PX4-Autopilot/build/px4_sitl_default`
* `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:{$path}/PX4-Autopilot`
* `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:{$path}/PX4-Autopilot/Tools/sitl_gazebo`
* `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{$path}/PX4-Autopilot/fsc_models`

5. Run `source ~/.bashrc`.

6. \* Go to `PX4-Autopilot` folder, create a folder `scripts`, put `rosbag_writer.py`, `rosbag_reader_estimation.py`, `aruco_estimation.py` and `aruco_combine_rotate.py` in `scripts`.

7. \* Move `single_drone_visual_sitl_QR_code_wind.launch` in `Google_Drive/PX4_Gazebo_Model/launch to $path/PX4-Autopilot/launch`.

8. \* Move `model single_iris_visual_QR_code_expanded` in `Google_Drive/PX4_Gazebo_Model/models to $path/PX4-Autopilot/Tools/sitl_gazebo/models`.

9. Create airframes named `single_iris_visual_QR_code_expanded` in ~/.ros/etc/init.d-posix/airframes respectively by entering the folder and running the folloiwing commands:
`cp 10016_iris 3016_single_iris_visual_QR_code_expanded`.

10. Apply the same airframe creating processes (step 8) in the following folders:
* `~/.ros/sitl_iris_0/etc/init.d-posix/airframes`
* `~/.ros/sitl_iris_1/etc/init.d-posix/airframes`
* `~/.ros/sitl_iris_2/etc/init.d-posix/airframes`

11. Run `roslaunch px4 single_drone_visual_sitl_QR_code_expanded.launch` for the no-wind case or `roslaunch px4 single_drone_visual_sitl_QR_code_wind.launch` for the case with the wind plugin applied, this launch file will launch mavros and start the rosbag writing process simultaneously.

12. Launch QGroundControl, click `Plan` at the upper-left corner, then click `File` -> `Storage` -> `Open` to load the QGroundControl path file (.plan file). Then, click `Upload` at the top of the window. After that, click `Fly` at the upper-left control bar to return to the main page.

13. To launch the drone, when there is a sliding component presenting at the bottom showing `Takeoff from ground and start the current mission`, slide to take-off. If not, go back to `Plan` and click `Upload` at the top of the window again and return to `Fly`.

14. After launching, the ground truth of the pose of the payload is stored in the topic: `/gazebo_ground_truth_payload`.

15. Run `python scripts/rosbag_reader_estimation.py` to draw diagrams of the results.

## Parameter modifications:
* Location of rosbag saving: `rosbag_writer.py`, Line 47: `self.bag_loc`
* Weight of payload: `single_iris_visual_QR_code_expanded.sdf`, Line 691, `<mass>${weight}</mass>`
* Wind speed: `PX4-Autopilot/Tools/sitl_gazebo/worlds/windy.world`, Line 20, `<windVelocityMean>${wind_speed}</windVelocityMean>`

## To generate new marker:
1. Go to `aruco_combine_rotate.py`: 

2. At Line 150, specify the absolute path that the generated marker image will store in the format: `.../final.png` 

3. At Line 151, the three parameters of create_dae() are 
* the absolute path to the proto .dae structure file (dae_struct.dae in the drive); 
* the absolute path to the generated .dae file; 
* the generated image, should be the same as Line 150. 
 
4. Then, run `python aruco_combine_rotate.py`

5. After finishing running the scripts, move the generated .dae file (second parameter of Line 151) to `PX4-Autopilot/Tools/sitl_gazebo/models/${model name}/meshes`

6. Also, move the generated image (parameter of Line 150) to `PX4-Autopilot/Tools/sitl_gazebo/models/${model name}/materials/textures`

In our case, ${model name} is single_iris_visual_QR_code_expanded, and it is recommended to set the final destinations (.../meshes and .../textures) of the .dae and .png files as the corresponding parameters in Line 150 and 151. 

## Location of files in drive:
Path to the Drive: https://drive.google.com/drive/folders/1bcr7QfcJM_zpQUhwNJ2592s2CpKBPATb
### Scripts
* Aruco generator: scripts/aruco_combine_rotate.py 
* Estimator: scripts/aruco_estimation.py
* Rosbag Writer: scripts/rosbag_writer.py
* Script for testing the results: scripts/rosbag_reader_estimation.py
### Rosbag
* Rosbag of Zigzag Path: rosbag/single_zigzag_estimation.bag
### Models
* Single drone model with payload: models/single_iris_visual_QR_code_expanded
### Launch File
* Launch file without wind: launch/single_drone_visual_sitl_QR_code_expanded.launch Launch file with wind: single_drone_visual_sitl_QR_code_wind.launch
### QGroundControl Path File
* Circular path file: paths/qgc_circle_alt.plan
* Zigzag path file: paths/qgc_zigzag_alt.plan
### Proto .dae
* Proto dae file: dae_struct.dae
