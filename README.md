# Logistic Planning for Emergency Services  

#### Keywords  
`Logistic Planning`, `Emergency Services`, `Agent-Based System`, `Classic Planning`, `Temporal Planning`, `Robotic Planning`, `PDDL`, `LPG-TD`, `Autonomous Agents`, `Artificial Intelligence`.  

This project models and implements a planning system for an agent-based approach to solve a logistics problem for emergency services. The objective is to design an AI system capable of **classic planning**, **temporal planning**, and **robotic planning** for autonomous agents. The scenario involves coordinating emergency services to reach injured individuals while managing the transportation of various objects, optimizing logistics under dynamic constraints.  

This project was developed as part of a degree coursework exam for the *Artificial Intelligence* course during the first year of the Master's degree in Computer Engineering.  

### Key Features  
- **Agent-Based System**: Utilizes autonomous agents to solve logistics challenges.  
- **Planning Approaches**: Includes classic planning, temporal planning, and robotic planning techniques.  
- **Scenario**: Simulates emergency services reaching injured individuals with varying transport requirements.  

### Project Structure  
- **classicalPlanning**:  
  - *classes*: Contains the planner implementation and a custom heuristic function.  
  - *pddl*: Includes domain and problem formulations for classic planning tasks.  
- **plansys2_project**:  
  - *CMakeLists.txt* and *package.xml*: Define dependencies and build configurations.  
  - *src*: Contains C++ files implementing simulated actions.  
  - *launch*: Holds the launch commands and scripts (`plansys2_project_launch.py`).  
  - *pddl*: Contains the domain and problem definitions for durative actions, along with plans generated using **LPG-TD**.  



