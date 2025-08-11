@struct_hash_equal struct RobotState
    id::Int
    pos::Tuple{Int, Int}
end


@struct_hash_equal mutable struct StateDec 
    id::Int
    robots_states::MVector
    gridmap::MMatrix
    known_cells::Int64
    seen_cells::Int64
    step::Int 
end


@struct_hash_equal struct ActionDec 
    direction::Tuple{Float64,Float64}
end


@struct_hash_equal mutable struct StateCen
    gridmap::MMatrix
    robots_states::Vector{RobotState}
    seen::Vector
    step::Int 
end


@struct_hash_equal struct ActionCen 
    directions_vector::Vector{ActionDec} 
end


struct RobotMDP <: MDP{Union{StateDec,StateCen}, Union{ActionDec,ActionCen}}
    vis_range::Int64
    nb_obstacle::Int64
    discount::Float64
    possible_actions::Union{Vector{ActionDec}, Vector{ActionCen}}
    reward_function::Function
    filtering_info::Bool
    max_depth::Int
end


@struct_hash_equal mutable struct RobotPlan 
    state::RobotState
    best_sequences::Vector{MutableLinkedList{ActionDec}}
    assigned_proba::Vector{Float64}
    timestamp::Int
end


struct AStarState
    pos::Tuple{Int, Int}
    gridmap::MMatrix
end 


mutable struct RolloutInfo
    timestamp_rollout::Int64 
    robots_plans::MVector
    in_rollout::Bool
    frontiers::Set
    route::Vector{AStarState}
    length_route::Int
    breakpoint::Vector{Int}
    last_best_action::ActionDec
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
    rollout_parameters::RolloutInfo
    state::StateDec
    planner::DPWPlanner
    last_comm::Int64
    last_action::ActionDec
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
    last_comm::Int64
end

mutable struct SharedMemory
    gridmap::MMatrix
    frontiers::Set
    plan::Vector{Vector{Tuple}}
    pathfinder::Any
end



Robot = Union{RobotCen,RobotDec,RobotPosMin}
