
Main tasks:

* Keeping regular journal:
	* Capturing research made & results => github

 - Finalizing proof of concept:
	 - **Cone detection algo**: Implementing performant 2D cone detection algorithm.
	 - **Control operations**: Basic robot-steering for going to a specific point in space.
	 - **SLAM implementation**: exploring RVIZ tool maps
	 - **Challenges**: Identifying the main challenges
 
 - Testing & Simulations:
	 - **Environments**: inside & outside, with wind, very light rain
	 - **Speed**: turtlebot can only go upto 5km/h, mimic higher speeds by using lower scanrates
	 - **Perturbations**: vibrations (detecting & supressing false negatives and positives by finetuning the parameters and implementing a lowpass filter according to needed sensitivity)
	 - **Cones**: Detecting the cone types (3D-printed model)

- Maybes (future):
	- Testing out virtual models of a 3D-lidar camera
	- Integration of a 2D LiDAR and camera for calibration and ++ 
	- Integration with IMU
	- Comparing pointcloud processing algos on the Kitty Dataset 