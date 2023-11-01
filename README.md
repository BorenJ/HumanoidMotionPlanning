
# Nao Humanoid Robot C++ Libraries

This project contains the model of Nao humanoid robot. For given joint angles, one can derive the kinematics model of the robot (end-effector positions, center of mass). Furthermore, conbined with C++ optimizer, Ipopt, one can do the whole-body trajectory planning. 

- **Creat ROS Workspace**
	Please kindly follow the instructions in [CreatRosWrokspace](https://github.com/BorenJ/ChernoCPPSeriesPractice). The codes are under the path of `robot_ws/src/NaoRobot/src`.
- **Add dependencies**
	The project depends on `Eigen` and Ipopt. Kindly make sure that in your ros workspace you can use [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page). In this project, the `eigen-3.4.0` is located at the path `robot_ws/src/NaoRobot`. For the Ipopt, please follow the [installation instructions](https://github.com/BorenJ/Ipopt_Ubuntu). 
- **Codes implementation**
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
	
	The initial pos and optimized pose of the robot are respectively
	
![Image text](https://github.com/BorenJ/HumanoidMotionPlanning/blob/main/img/pos0.png)

![Image text](https://github.com/BorenJ/HumanoidMotionPlanning/blob/main/img/pos1.png)	


- **Codes explanation**
	The optimizaer is Ipopt. We have two hearder files `hs071_nlp.hpp` and `nao.hpp`, two source files `hs071_nlp.cpp` and `nao.cpp`, and one main file `Nao.cpp`.
	
	`hs071_nlp.hpp` is the optimizer hearder file. It declares the function and class of the optimization. The details is in [Ipopt website](https://coin-or.github.io/Ipopt/INTERFACES.html#INTERFACE_CPP). Some differences should be noticed. 
	1. The Jacobian and Hessian matrix in our code are not provided mannually. They are 		computed by default finite difference in Ipopt. So the Hessian method in the class is not declared in the header file (being commented out).
	2. The optimization parameters and functions, such as target CoM, initial guess, cost function, constrain function are declared at the beginning of the header file as 
		```
		extern  Eigen::VectorXd  q_init;
		extern  double  com_goal[2];
		extern  Number  constrain[1];
		Number  opt_fun(const  Number*  x, NaoRobot&  nao, double*  com_goal);
		Number*  opt_constrain(Number*  constrain, const  Number*  x,NaoRobot& nao, const  Eigen::VectorXd  q_init); 
		```
		
	`hs071_nlp.cpp` is the optimizer source file. It implements methods declared in the header file. Specificlly, eight pure virtual methods of the optimization must be implemented in this source file. Since we do not provide analytical Jacobian and Hessian for the cost function and constrains, we directly `return true` in the function `eval_grad_f`.

	`nao.hpp` declares the class of Nao humanoid robot and math function, such as rotation matrix. Two functions with template are also implemented in this source file. The Nao class contains the member of Nao robot dunamic model parameters and joint consrains, and also two methods of calculating forward kinematics and center of mass of Nao.
	
	`nao.cpp` implements the functions and class declared in the `nao.hpp`.  The Eigen library is used here to store the parameters of the robot.

	`Nao.cpp` is the main function file. It first set the target CoM and initial posture of the humanoid  robot. Then, calls the optimization functions in Ipopt. Since we do not provide the Jacobian and Hessian to the optimizer, we need to set 
	```
	app->Options()->SetStringValue("gradient_approximation", "finite-difference-values");
	app->Options()->SetStringValue("hessian_approximation", "limited-memory");
	app->Options()->SetStringValue("jacobian_approximation", "finite-difference-values");
	```
	This can let Ipopt numerically calculate the Jacobian and Hessian. 




