
# Nao Humanoid Robot C++ Libraries

This project contains the model of Nao humanoid robot. For given joint angles, one can derive the kinematics model of the robot (end-effector positions, center of mass). Furthermore, conbined with C++ optimizer, Ipopt, one can do the whole-body trajectory planning. 

- **Creat ROS Workspace**
	Please kindly follow the instructions in [CreatRosWrokspace](https://github.com/BorenJ/ChernoCPPSeriesPractice). The codes are under the path of `robot_ws/src/NaoRobot/src`.
- **Add dependencies**
	The project depends on `Eigen` and Ipopt. Kindly make sure that in your ros workspace you can use [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page). For the Ipopt, please follow the [installation instructions](https://github.com/BorenJ/Ipopt_Ubuntu). In this project, the `eigen-3.4.0` is located at the path `robot_ws/src/NaoRobot`.
- **Codes explanation and implementation**
		Please download all the code files to the path `robot_ws/src/NaoRobot/src`. 
		The `CMakeLists.txt` should contain follows:
	```
	set(CMAKE_CXX_STANDARD 17)
	
	include_directories(
	"/home/boren/Desktop/robot_ws/src/NaoRobot/eigen-3.4.0"
	${catkin_INCLUDE_DIRS}
	"/usr/local/include/coin-or/"
	)
	
	add_executable(naoRobot src/Nao.cpp src/nao.cpp src/hs071_nlp.cpp)
	
	target_link_libraries(naoRobot
	${catkin_LIBRARIES}
	"/usr/local/lib/libipopt.so"
	)
	```
	In VSCode terminal,
	 ```
	source ./devel/setup.bash
	echo $LD_LIBRARY_PATH
	export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
	rosrun NaoRobot naoRobot
	 ```
    
	This code is to find an optimal robot posture where the center of mass is on the target. The goal CoM can be set in the `Nao.cpp` as
	```
	com_goal[0] = 20.15;
	com_goal[1] = 13.4;
	```
	The initial posture of the robot is 
	```
	q_init  << 0, -20, 0, 49, -29, 0, -20, 0, 49, -29, 0;
	```
	After this, a list of joint angles (robot leg angels) will be displayed in the VScode terminal. To visualize the robot posture, one can input it to the [Nao visualization MATLAB code](https://github.com/BorenJ/NaoSelfCalibration).  
![Image text](https://github.com/BorenJ/HumanoidMotionPlanning/blob/main/img/pos0.png)




