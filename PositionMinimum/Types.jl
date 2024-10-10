@struct_hash_equal struct robot_state
    id::Int
    pos::Tuple{Int, Int}
end

@struct_hash_equal mutable struct space_state
    id::Int
    gridmap::MMatrix
    known_cells::Int64
    seen_cells::Int64
    robots_plans::MVector # vector of Robot_plan
end

@struct_hash_equal struct mdp_state 
    space_state::space_state
    nb_coups::Int
end


@struct_hash_equal struct action_robot
    action::Tuple{Float64,Float64}
end


struct robotMDP <: MDP{mdp_state, action_robot}
    vis_range::Int64
    nb_obstacle::Int64
    discount::Float64
    possible_actions::Vector{action_robot}
end


@struct_hash_equal mutable struct Robot_plan 
# mutable struct Robot_plan 
    state::robot_state
    best_sequences::Vector{Vector{action_robot}}
    assigned_proba::Vector{Float64}
end


mutable struct Rollout_info
    in_rollout::Bool
    debut_rollout::Int64
end


mutable struct Robot{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Int}
    vis_range::Int
    com_range::Int
    isObstacle::Bool
    # plans::Union{Nothing, Vector{Robot_plan}}
    rollout_info::Union{Nothing, Rollout_info}
    state::Union{Nothing, mdp_state}
    planner::Union{Nothing, DPWPlanner}
end


# mutable struct KosarajuGraph
#     nodes::MMatrix
# end