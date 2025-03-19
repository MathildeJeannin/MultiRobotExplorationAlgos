# Multi robot exploration contro algorithms

Dev version. Goal = add frontier method in DecMCTS control algorithm. 

## Description of the algorithms

This package implements four control algorithms for robotic explorations of unknown environment. It is based on the package [Agents.jl](https://juliadynamics.github.io/Agents.jl/stable/), [POMDPs.jl](https://juliapomdp.github.io/POMDPs.jl/stable/) and [MCTS.jl](https://juliapomdp.github.io/MCTS.jl/stable/). 

The environment is represented by a discrete gridmap. The four algorithms are : 

1. *CenPositionMinimum*: a centralised frontier method

In this method, all robots share the gridmap, a frontier set and a A* pathfinding. The frontiers are allocated using [Bautin et al.](https://link.springer.com/chapter/10.1007/978-3-642-33515-0_49) strategy. 


2. *DecPositionMinimum*: a decentralised frontier method

This method is similar as the previous one, but each robot keeps its own gridmap and frontier set. Each robot takes an action using the last information it got from other robots.

3. *CenMCTS*: a centralised method using a shared MCTS

This method uses a Monte Carlo Tree Search (MCTS) shared by all robots along the gridmap. The Search Tree decides the actions of the robots.

4. *DecMCTS*: a decentralised method using one MCTS per robot

This method is a decentralised version of the previous one. It is based on [Best et al.'s work](https://journals.sagepub.com/doi/abs/10.1177/0278364918755924]).



## Installation of the packages 

In Julia type `include("/src/Packages.jl")`

## Usage


Open a Julia terminal. In the DecMCTS case, open a terminal using `julia --threads nb_robots`, nb_robots being the number of robots you want because the the DecMCTS parallelises the MCTS searches. When a Julia terminal is open, type 
```julia 
include("./algo/Run.jl")
nb_steps = run()
``` 
algo being the name of the algorithm that you want to test. It returns the number of steps needed to visit the whole environment. 

To keep a log of the exploration, type 
```julia
nb_steps = run(id_expe = 1, file = "file.csv")
```

The following parts regroup all possible keywords od the `run()` function.  


### Other run parameters

#### Global parameters: 

The user can chose the number of robots `nb_robots=5`, the maximum number of steps for the simulation `max_steps=500`, the possibility of visualise the figure `vis_figure=true` and to the see the progress of the MCTS searches `show_progress=true`. The robots vision range and communication range can be changed using `vis_range=3, com_range=10`.


#### Environment:
The argument keyword to chose a map is `num_map`.
They are 5 maps representing either indoor (maps 1, 2 and 4) or outdoor maps (3 and 5) of different sizes, called extent in this work (15x15, 20x20, 40x40). This work also generates maps with N_obs random obstacles and desired size (X,Y) using the keywords `num_map=0, nb_obstacles=[N_obs], extent=(X,Y)`, or maps with obstacles in form of blocks using keywords `num_map=-1, nb_blocs=N_bloc, extent=(X,Y)`. The robots start in a random position x,y with x (respectively y) in [1,Bx]: `begin_zone=(Bx,By). ` 



#### MCTS parameters

This work uses a variant of the MCTS called Double Progressive Widening (DPW) allowing to visit the tree in depth even if the previous layer of the tree is not fully expanded. See the [MCTS.jl doc](https://juliapomdp.github.io/MCTS.jl/stable/dpw/) for more information. The DPW parameters are `alpha_state = 1.0, k_state = 1.0, alpha_action = 3.0, k_action = 1.0`. The MCTS uses a Upper Confidence Bound (UCB) as tree policy and its parameters are `exploration_constant=20.0`. The number of rollouts during the MCTS is `n_iterations=500`, the discount is `discount=0.85`, the depth is `depth=100`. The applied depth is the maximum between the parameter `depth` and `5*maximum(extent)`. 


#### Example: 

```julia
include("./DecMCTS/Run.jl")
nb_steps = run(nb_robots=5, num_map=-1, nb_blocs=8, extent=(40,40), com_range=10, vis_range=5, vis_figure=true, show_progress=true)
```