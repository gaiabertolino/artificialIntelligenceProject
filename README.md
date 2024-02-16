# artificialIntelligenceProject

The project models a problem domain for a system based on autonomous agents to which classic planning, time planning and robotic planning are then applied. In particular, the scenario represents a problem of logistic organization in which a number of injured people must be reached by emergency services, which can transport various objects.
The attached files are organized in the following structure:

In the folder "classicalPlanning" there are 
- the classes subfolder containing the planner implementation classes beloging to the custom heuristic function
- the pddl subfolder containing the domain formulation and domain formulation files problems of the second task.

In the folder "plansys2_project" there are
- the CMakeLists.txt file and the package.xml file that reports dependencies
- the subfolder src that reports C++ files that perform fake actions
- the launch subfolder that contains the commands file and the launch file plansys2_project_launch.py
- the pddl subfolder which contains the domain and the problem with durative actions and the plan obtained from the application of LPG-TD
