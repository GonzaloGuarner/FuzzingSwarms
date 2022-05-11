# Welcome to the Fuzzing Swarms repository!

The Fuzzing Swarms are a FIS-based swarm robotics system optimized through the use of PSO. In this project the Fuzzing Swarms are developed and trained to conduct a deployment task in an unknown environment.

####Here I will detail the:

* Main scripts and their utilities

* Required toolboxes.

* Necessary toolbox script changes if using custom maps

## Scripts

DeploymentSimulation.m is the simplest fuzzing swarms scripts, it simulates a swarm deployment task and shows the behaviour of the agents in the environment of your choice.

DeploymentMeasurement.m is an extension to DeploymentSimulation, it's main goal being measuring performance across several simulations or 'runs'. Support to sensor noise and agent failure is also implemented.

PSOtuneFISmultipleArenas.m is the core of this project. This script optimizes a fuzzy inference system through particle swarm optimization. The particles in PSO represent a type of FIS, and their fitness is obtained through their performance in a deployment task, across multiple arenas.

RandomWalk.m was used for comparison purpouses with a random walk approach. 


## Required Toolboxes & Add-Ons

* Fuzzy Logic Toolbox v2.9
* Robotics System Toolbox v4.0
* Optimization Toolbox v9.3
* Navigation Toolbox v2.2
* Global Optimization Toolbox v4.7
* Mobile Robotics Simulation Toolbox v2.2. Available in the Add-Ons page.

## Necessary toolbox script changes

To pass custom arenas to our multi-robot environment, some editions are necessary in the 
MultiRobotEnv script (found in the Mobile Robotics Simulation Toolbox).

Declare as properties (Nontunable) customMap and hasCustomMap


Line 171 change:

> obj.map = internal.createMapFromName(obj.mapName);            

for:

> if obj.hasCustomMap

obj.map = obj.customMap;

else
	
obj.map = internal.createMapFromName(obj.mapName);

end