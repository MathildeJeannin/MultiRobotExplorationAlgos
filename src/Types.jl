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
    direction::Union{Tuple{Float64,Float64}, Tuple{Int64,Int64}} #changer nom
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



mutable struct RolloutInfo
    timestamp_rollout::Int64 
    robots_plans::MVector
    pathfinder::Any
    frontiers::Set
    route::Any
end


struct RobotMDP <: MDP{Union{StateDec,StateCen}, Union{ActionDec,ActionCen}}
    vis_range::Int64
    nb_obstacle::Int64
    discount::Float64
    possible_actions::Union{Vector{ActionDec}, Vector{ActionCen}}
    reward_function::Function
    use_old_info::Bool
end


# mutable struct Ghost{D} <: AbstractAgent
#     id::Int
#     pos::NTuple{D,Int}
# end


@struct_hash_equal mutable struct RobotPlan 
    state::RobotState
    best_sequences::Vector{MutableLinkedList{ActionDec}}
    assigned_proba::Vector{Float64}
    timestamp::Int
end


mutable struct Buffer
    # best_sequences::Vector{MutableLinkedList{ActionDec}}
    # assigned_proba::Vector{Float64}
    # position::Tuple
    # gridmap::MMatrix
    # frontiers::Set
    # empty::Bool
    best_sequences
    assigned_proba
    position
    gridmap
    frontiers
    empty
end


mutable struct RobotDec{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Int}
    vis_range::Int
    com_range::Int
    plans::Vector{RobotPlan}
    rollout_parameters::RolloutInfo
    state::StateDec
    pathfinder::Any #ajout 
    frontiers::Vector{Set}
    planner::DPWPlanner
    last_comm::Int64
    buffers::Vector{Buffer}
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
