@struct_hash_equal struct robot_state
    id::Int
    pos::Tuple{Int, Int}
end


@struct_hash_equal struct mdp_state 
    robot_list::Vector{robot_state}
    gridmap::MMatrix
    seen_gridmap::MMatrix
    nb_coups::Int
end


@struct_hash_equal struct action_robot
    id::Int
    action::Tuple{Float64,Float64}  
end


struct robotMDP <: MDP{mdp_state, Vector{action_robot}}
    nb_robots::Int
    vis_range::Int
    nb_obstacle::Int
    discount::Float64
    possible_actions::Vector{Vector{action_robot}}
end

mutable struct Robot{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Int} # pos and vel required like this for ContinuousAgent
    vis_range::Int  # how far can it see?
    com_range::Int  # how far can it communicate?
    isObstacle::Bool
end