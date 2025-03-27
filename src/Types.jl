@struct_hash_equal struct RobotState
    id::Int
    pos::Tuple{Int, Int}
end


@struct_hash_equal mutable struct StateDec #changé avant : State
    id::Int
    robots_states::MVector
    gridmap::MMatrix
    known_cells::Int64
    seen_cells::Int64
    step::Int #changé avant : nb_coups
end


@struct_hash_equal struct ActionDec #changé avant : Action
    direction::Tuple{Float64,Float64}
end


@struct_hash_equal mutable struct StateCen
    gridmap::MMatrix
    robots_states::Vector{RobotState}
    seen::Vector
    step::Int #changé avant : nb_coups
end


@struct_hash_equal struct ActionCen 
    directions_vector::Vector{ActionDec} #changé avant : directions
end


struct RobotMDP <: MDP{Union{StateDec,StateCen}, Union{ActionDec,ActionCen}}
    vis_range::Int64
    nb_obstacle::Int64
    discount::Float64
    possible_actions::Union{Vector{ActionDec}, Vector{ActionCen}}
    reward_function::Function
    use_old_info::Bool
end


@struct_hash_equal mutable struct RobotPlan 
    state::RobotState
    best_sequences::Vector{MutableLinkedList{ActionDec}}
    assigned_proba::Vector{Float64}
    timestamp::Int
end


mutable struct RolloutInfo
    timestamp_rollout::Int64 #changé : avant = debut_rollout
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
