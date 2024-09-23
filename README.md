# Multi-UAV Path Planning for Urban Air Mobility

## Table of Contents
- [Introduction](#introduction)
- [Abstract](#abstract)
- [Usage](#usage)
- [Methodology](#methodology)
- [Results](#results)
- [Conclusion and Future Scope](#conclusion-and-future-scope)
- [Acknowledgements](#acknowledgements)
- [License](#copyright_declaration_and_license)

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
rather than pre-planned, offline algorithms. To this end, this project presents a framework for 3D
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
and locally replanning path segments where potential collisions are detected. The algorithm’s performance was validated in two distinct real-world urban environments.<br>
**Keywords**: Unmanned Aerial Vehicle (UAV); multi-UAV; Path Planning; Collision avoidance;
RRT*

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

4. Run the Demo file and follow the instructions to generate paths, as shown in the following figures:
<p align="center">
  <img src="Figures/Demo/Demo_1.png" alt="1" width="200"/>
  <img src="Figures/Demo/Demo_2.png" alt="2" width="200"/>
  <img src="Figures/Demo/Demo_3.png" alt="3" width="200"/>
  <img src="Figures/Demo/Demo_3_results.png" alt="4" width="200"/>
</p>

    
## Methodology
### Mapping
Two urban areas, New York City and London, were selected for testing and implementation. Using OpenStreetMap data, a UAV scenario was created for each area, and a 3D occupancy map was generated by simulating Lidar scans. These scans were then inflated to ensure a safety margin from nearby buildings. The steps for generating the 3D occupancy map for the selected areas are shown in the following figures:
<p align="center">
  <img src="maps/Figures/NYC_google_earth4.png" alt="Real Map" width="200"/>
  <img src="maps/Figures/NYC_Scenario.png" alt="UAV Scenario" width="200"/>
  <img src="maps/Figures/NYC_3Docc.png" alt="3D Occupancy Map" width="200"/>
  <img src="maps/Figures/NYC_3Docc_inflated10.png" alt="Inflated Occupancy Map" width="200"/>
</p>
<p align="center">
  <img src="maps/Figures/lidar_ICL.gif" alt="a" width="400"/>
  <img src="maps/Figures/Omap_ICL.gif" alt=" B" width="400"/>
</p>
The resulting 3D occupancy maps and UAV scenarios can be found in [maps](maps) To get the 3D occupancy map and the UAV scen for a different area, use the function [Get_3DPathPlanningEnv](maps/Get_3DPathPlanningEnv.m). 
### Single UAV Path Planning 
Several improvements were made to achieve an optimised UAV path frpm the RRT\* algorithm.
As shown in figures (a) and (b), computational effort, linked to the size of the random tree, was reduced by 
restricting the sampling range to a variable rectangle based on the start and target points. Redundant 
waypoints were removed (c), and the process was repeated with different random seeds to find the 
shortest path(d). Finally, the path was smoothed to align with UAV dynamics (e). The improved RRT\* algorithm can be found in the function [One_Path_Planning](Path_Planning_Functions/One_path_planning.m). 

<table>
  <tr>
    <td align="center">
      <img src="Figures/Improved_RRT*/non-restricted.png" alt="non-restricted" width="200"/><br>
      (a) Full Map Sampling Range
    </td>
    <td align="center">
      <img src="Figures/Improved_RRT*/restricted.png" alt="restricted" width="200"/><br>
      (b) Reduced Sampling Range
    </td>
    <td align="center">
      <img src="Figures/Improved_RRT*/Pruned path.png" alt="Pruned path" width="200"/><br>
      (c) Pruned Path
    </td>
    <td align="center">
      <img src="Figures/Improved_RRT*/shortest_path_e.png" alt="shortest_path_e" width="200"/><br>
      (d) Shortest Path
    </td>
    <td align="center">
      <img src="Figures/Improved_RRT*/smoothed_path.png" alt="smoothed_path" width="200"/><br>
      (e) Smoothed Path
    </td>
  </tr>
</table>
   


### Multi-UAV Path Planning 
When planning for multiple UAVs, an additional complexity arises : maintaining the required safety 
distance between them. To avoid any potential collision, the following path-editing algorithm is applied :

<p align="center">
  <img src="Figures/algo_diagram.png" alt="algo" width="500"/>
</p>

Further details on the project can be found in [MultiUAV Path Planning for Urban Air Mobility](MultiUAV_Path_Planning_for_Urban_Air_Mobility.pdf)

## Results
As shown below, after applying the two stages of the algorithm—initial path planning and path editing—each UAV maintains the required safety distance from all other UAVs at every time step of its trajectory.

<table>
  <tr>
    <td align="center">
      <img src="Figures/Multi_UAVs_Paths/8_UAVs_NYC_original.png" alt="full map sampling range" width="300"/><br>
      (a) Initial Paths
    </td>
    <td align="center">
      <img src="Figures/Multi_UAVs_Paths/8_UAVs_NYC_corrected.png" alt="reduced sampling range" width="300"/><br>
      (b) Edited Paths
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="Figures/Multi_UAVs_Paths/distances_plot_original_paths.png" alt="pruned path" width="300"/><br>
      (c)Initial Paths
    </td>
    <td align="center">
      <img src="Figures/Multi_UAVs_Paths/distances_plot_corrected_paths.png" alt="shortest path" width="300"/><br>
      (d) Edited Paths
    </td>
  </tr>
</table>


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
 I would like to express my gratitude to Dr Giordano Scarciotti, Dr Martina Sciola and Dr Roberto Valenti for their invaluable guidance throughout this project.

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

## References : 
 [1] S. A. H. Mohsan, M. A. Khan, F. Noor, I. Ullah, and M. H. Alsharif, “Towards the unmanned
 aerial vehicles (uavs): A comprehensive review,” Drones, vol. 6, no. 6,-06-15 2022. doi:
 10.3390/drones6060147.<br>
 [2] G. Skorobogatov, C. Barrado, and E. Salamí, “Multiple uav systems: A survey,” Unmanned
 Systems, vol. 08, no. 02, p. 149,-04-02 2020. doi: 10.1142/s2301385020500090.
