# Multi-UAV Path Planning for Urban Air Mobility

## Table of Contents
- [Introduction](#introduction)
- [Abstract](#abstract)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Methodology](#methodology)
- [Results](#results)
- [Conclusion and Future Scope](#conclusion-and-future-scope)
- [Acknowledgements](#acknowledgements)
- [License](#license)

## List of Acronyms

- **RRT**: Rapidly-exploring Random Tree
- **RRT\***: Rapidly-exploring Random Tree Star
- **UAV**: Unmanned Aerial Vehicle

## Introduction
Unmanned aerial vehicles are widely used as platforms to work in various environments. They have
a wide range of applications, such as search and rescue, parcel delivery, hidden area exploration and
many others [1]. The main advantages of applying these systems are their flexibility, simultaneous
actions, time efficiency and low cost [2]. Motion planning represents a fundamental step in the operation of autonomous Unmanned Aerial
Vehicles (UAVs). While significant progress has been made in addressing the challenges of UAV path planning, much of the existing research focuses on simplified
scenarios rather than real-world urban environments. Additionally, most studies do not consider
scenarios involving multiple UAVs, and those that do often rely on real-time, reactive avoidance
rather than pre-planned, offline algorithms. To this end, this thesis presents a framework for 3D
path planning in urban environments for multiple UAVs. This work is distinguished by introducing a new RRT*-based collision-free path planning algorithm for multiple UAVs implemented on
real-world urban maps.
## Abstract
The implementation environments used in this work are set up by creating 3D occupancy maps
and 3D UAV scenarios derived from urban maps. An enhanced Rapidly-exploring RandomTree
Star (RRT*) algorithm was developed to overcome the limitations of the basic RRT* algorithm,
specifically its poor directionality in path planning and the unsuitability of its paths for UAV
flight. First, the sampling area was constrained, improving the path search efficiency. Then, a
novel method was introduced to prune the generated paths by removing redundant waypoints.
The shortest possible path was then identified by running the planning algorithm multiple times
with different random seeds. Finally, a path smoothing algorithm using cubic splines was applied
to optimise UAV flight paths. Simulation results demonstrate that the improved RRT* algorithm
reduces path planning time and produces more optimised paths for UAVs. The improved single
UAV path planning algorithm was extended to generate paths for multiple UAVs. An original editing algorithm was proposed for multi-UAV path planning, designed to maintain a minimum safety
distance between UAVs throughout their trajectories. This is achieved by placing pseudo-obstacles
and locally replanning path segments where potential collisions are detected. The algorithm’s performance was validated in two distinct real-world urban environments.
**Keywords**: Unmanned Aerial Vehicle (UAV); multi-UAV; Path Planning; Collision avoidance;
RRT*

## Installation
To set up the project, follow these steps:

1. Clone the repository:
    ```bash
    git clone https://github.com/your-username/robust-visual-slam.git
    ```
2. Navigate to the project directory:
    ```bash
    cd robust-visual-slam
    ```
3. Install the required software:
    - MATLAB

## Usage

### Tools Required:
- MATLAB
- UAV Toolbox
- Navigation Toolbox
- Sensor Fusion and Tracking Toolbox

### Steps to Use:

1. **Download Required Tools:**
   Ensure all necessary MATLAB toolboxes are installed.

2. **Download and Extract Zip File:**
   Download the project files and extract them.

3. **Open Folder in MATLAB:**
   Open the extracted folder in MATLAB.

4. 
   
5. 
6. 
7. 
8. 
9. 
## Methodology

### Software
- MATLAB

### Workflow

## Results


### Figures and plots


## Conclusion and Future Scope

This project presented a novel framework for 3D path planning in urban environments for Unmanned Aerial Vehicles (UAVs), focusing on offline path planning for multiple UAVs in real urban scenarios. An enhanced RRT\*-based algorithm was developed to address several limitations of the traditional RRT\* algorithm, such as poor directionality and non-smooth paths, making the generated paths more suitable for UAV flight in urban settings.

Key improvements to the algorithm include:
- Constraining the sampling area.
- Pruning redundant waypoints.
- Finding the shortest path.
- Smoothing the generated path.

These enhancements optimized the paths for UAV navigation. In addition, an original path editing algorithm was introduced, leveraging pseudo-obstacles to maintain a minimum safety distance between UAVs during flight. This algorithm was validated through simulations in two different real-world urban environments, successfully generating collision-free trajectories for multiple UAVs, even in dense urban areas.
### Further Work 
Integrating an avoidance algorithm to handle unexpected and dynamic obstacles that may appear during flight.
Additionally, the current implementation of the algorithm is computationally expensive, particularly when scaling to larger maps and handling multiple potential collisions. This opens up avenues for future research and further optimization to enhance scalability and efficiency.


## Acknowledgements
I would like to express my gratitude to Dr Martina Sciola and Dr Roberto Valenti for their invaluable guidance throughout
this project. Special thanks to all contributors and supporters who made this project possible.

## Copyright Declaration and License 
The copyright of this project rests with the author and is made available under a [Creative Commons Attribution Non-Commercial No Derivatives](https://creativecommons.org/licenses/by-nc-nd/4.0/) license. Researchers are free to copy, distribute, or transmit the work under the following conditions:

- **Attribution**: You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so reasonably, but not in any way that suggests the author endorses you or your use.
- **Non-Commercial**: You may not use the material commercially.
- **No Derivatives**: You may not distribute the modified material if you remix, transform, or build upon it.

For any reuse or redistribution, researchers must clearly state the terms of this license.

Additionally, this project is licensed under the [BSD 2-Clause License](LICENSE). The terms are as follows:

- **Redistribution and use** in source and binary forms, with or without modification, are permitted, provided that the following conditions are met:
  1. **Redistributions of source code must retain** the above copyright notice, this list of conditions, and the following disclaimer.
  2. **Redistributions in binary form must reproduce** the above copyright notice, this list of conditions, and the following disclaimer in the documentation and/or other materials provided with the distribution.

This software is provided "as is" without any express or implied warranties, including, but not limited to, implied warranties of merchantability or fitness for a particular purpose. See the [LICENSE](LICENSE) file for more details.
