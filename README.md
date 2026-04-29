[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/L0glmnMn)
[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-2e0aaae1b6195c2367325f4f02e2d04e9abb55f0b24a779b69b11b9e10269abc.svg)](https://classroom.github.com/online_ide?assignment_repo_id=23761750&assignment_repo_type=AssignmentRepo)
# RMPC-lab1

First laboratory work for course Robot Motion Planning and Control.

Assignment

1. For the selected robot option, load the manipulator model from the toolbox **different then Puma560** and output the Denavit-Hartenberg parameters.
2. Specify the masses of the links, the positions of their centers of mass, the tensors of inertia, the moments of inertia of the drives, the coefficients of viscous friction of the drives, the coefficients of Coulomb friction of the drives, the gear ratios of the reducers and the constraints on the generalized coordinates. It is recommended to refer to the data sheets of the manipulators used.
3. Specify and display arbitrary initial and final configurations of the robot.
4. Plan a trajectory between the specified configurations.
5. Solve the inverse dynamics problem using the Newton-Euler method for the following scenarios: <br>
+ non-zero velocities and accelerations $\dot{q} \neq 0, \ddot{q} \neq 0$;
+ non-zero velocities and negligible accelerations $\dot{q} \neq 0, \ddot{q} \approx 0$ - quasi-statics;
+ zero velocities and accelerations $\dot{q} = 0, \ddot{q} = 0$ - maintaining a given position;
6. Determine the numerical values ​​of the elements of the matrices $M(q), C(q, \dot{q}), G(q)$ at each calculated moment of time.
7. Plot graphs of the moments of the manipulator links along the trajectory for each scenario (see point #5).
8. Create a report in .ipynb format, with detailed comments.
9. All steps and conclusions are formulated based on the results of the work in README.md
  
