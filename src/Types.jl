@struct_hash_equal struct RobotState
    id::Int
    pos::Tuple{Int, Int}
end

# @struct_hash_equal mutable struct space_state
#     id::Int
#     gridmap::MMatrix
#     known_cells::Int64
#     seen_cells::Int64
#     robots_plans::MVector # vector of Robot_plan
# end

@struct_hash_equal mutable struct State 
    id::Int
    # space_state::space_state
    gridmap::MMatrix
    known_cells::Int64
    seen_cells::Int64
    robots_plans::MVector # vector of Robot_plan
    frontiers::Set
    nb_coups::Int
end


@struct_hash_equal struct Action
    direction::Tuple{Float64,Float64}
end


struct RobotMDP <: MDP{State, Action}
    vis_range::Int64
    nb_obstacle::Int64
    discount::Float64
    possible_actions::Vector{Action}
end


@struct_hash_equal mutable struct RobotPlan 
# mutable struct Robot_plan 
    state::RobotState
    best_sequences::Vector{Vector{Action}}
    assigned_proba::Vector{Float64}
end


mutable struct RolloutInfo
    in_rollout::Bool
    debut_rollout::Int64
end


mutable struct Obstacle{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Int}
end


mutable struct RobotCen{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Int}
    vis_range::Int
end


mutable struct RobotDec{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Int}
    vis_range::Int
    com_range::Int
    plans::Vector{RobotPlan}
    rollout_info::RolloutInfo
    state::State
    planner::DPWPlanner
    pathfinder::Any
end


mutable struct RobotPosMin{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Int}
    vis_range::Int
    com_range::Int
    gridmap::MMatrix
    all_robots_pos::Vector
    pathfinder::Any
    plan::Vector
    frontiers::Set
end

mutable struct SharedMemory
    gridmap::MMatrix
    frontiers::Set
    plan::Vector{Vector{Tuple}}
    pathfinder::Any
end

Robot = Union{RobotCen,RobotDec,RobotPosMin}
