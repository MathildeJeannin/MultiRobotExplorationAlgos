@struct_hash_equal struct RobotState
    id::Int
    pos::Tuple{Int, Int}
end


@struct_hash_equal mutable struct State 
    id::Int
    robots_states::MVector
    gridmap::MMatrix
    known_cells::Int64
    seen_cells::Int64
    nb_coups::Int
end


@struct_hash_equal struct Action
    direction::Tuple{Float64,Float64}
end


@struct_hash_equal mutable struct StateCen
    gridmap::MMatrix
    robots_states::Vector{RobotState}
    nb_coups::Int
end


@struct_hash_equal struct ActionCen
    directions::Vector{Action}
end


struct RobotMDP <: MDP{Union{State,StateCen}, Union{Action,ActionCen}}
    vis_range::Int64
    nb_obstacle::Int64
    discount::Float64
    possible_actions::Union{Vector{Action}, Vector{ActionCen}}
end


@struct_hash_equal mutable struct RobotPlan 
    state::RobotState
    best_sequences::Vector{MutableLinkedList{Action}}
    assigned_proba::Vector{Float64}
end


mutable struct RolloutInfo
    in_rollout::Bool
    debut_rollout::Int64
    frontiers::Set
    goal::Union{Tuple,Vector{Tuple{Int,Int}}}
    robots_plans::MVector
end


mutable struct Obstacle{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Int}
end


mutable struct RobotCen{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Int}
    vis_range::Int
    pathfinder::Any
    frontiers::Set
end


mutable struct RobotDec{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Int}
    vis_range::Int
    com_range::Int
    plans::Vector{RobotPlan}
    rollout_parameters::RolloutInfo
    state::State
    planner::DPWPlanner
    pathfinder::Any
    frontiers::Set
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
