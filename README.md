# Proyecto de Robótica
- Universidad de Extremadura. 
- Grado en Ingeniería Informática en Ingeniería de Computadores.
- Grupo: Juan Antonio Silva Luján y Rodrigo Puerto Pedrera.
- Curso: 2019/2020.
## Introduction to component-oriented programming and robot control.
Components are the best "marriage" between objects and processes. A brief introduction to component-oriented programming can be found here

The RoboComp robotics framework, mainly written by RoboLab's members,  will be introduced and explained. It is installed in all classroom computers. There are several tutorials that you can follow to learn how to create new components and how to connect them and the the simulated robot in the RCIS simulator and in the real Robex robot. 
### Goals:
1. Following the installation guide in RoboComp, start the RCIS simulator with the world file "simpleworld.xml". Check that it works.
2. Connect the JoyStick and move the robot around. See how on board camera is rendered.
3. Read tutorials:
    1. https://github.com/robocomp/robocomp/blob/stable/doc/components.md
    2. https://github.com/robocomp/robocomp/blobstable/doc/interfaces/README.md
    3. https://github.com/robocomp/robocomp/blob/stable/doc/using_github.md
    4. https://github.com/robocomp/robocomp/blob/stable/doc/robocompdsl.md
4. Following the last tutorial, create a new component, named "Controller" with the robocompdsl editor. 
5. This component will drive the robot among the obstacles avoiding collisions. The example code has to be improved to achieve a good fluent movement of the robot. Find out what makes the robot get stuck and write a solution.
6. Check the use of new C++14 features in the example code, such as the use of lambda functions in the std::sort third parameter.
7. This week's delivery will be ready when the robot can move for more than 10 minutes without getting stuck or hitting the obstacles.
8. Code simplicity, elegance and the way robot moves will contribute to the final grade

### Deliveries:
1. Create a new folder in your GitHub account for the component named "practica2" and  upload the new component when finished. Remember to upload only the sources.
2. OPTIONAL: Write a short paper (2 pages in English) using OverLeaf and  the IEEEConference template, on component oriented programming. Analyze the differences between Object Oriented Programming and Component Oriented Programming. When done upload the PDF file to the "campusvirtual".
3. Upload on due date a short paper describing the improvements done to the initial code. Describe each contribution, the reason why you did it and the expected results on the robot.
4. SELF-EVALUATION: Clone this repository: https://github.com/pbustos/beta-robotica-class in ~/robocomp/components/ and cd to the folder "aspirador". Compile the component and execute it after RCIS and before your component. This component wll watch your robot and measure the porcentage of floor that has been swept.
    1. Make sure that the maximum advance speed is limited to 1000 mm/sg and that the maximum rotational speed is set to 2 rads/sg
    2. Run your component for a period of 5 minutes and kill it.
    3. Check the porcentage of swept floor in the "aspirador" terminal and note it down.
    4. Repite this process 5 times
    5. Upload to the campus the mean of the five runs, clearly written in the short paper describing your improvements.
