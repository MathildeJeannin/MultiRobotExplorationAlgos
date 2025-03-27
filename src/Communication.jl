function merge_gridmaps!(r1::RobotDec, r2::RobotDec)
    extent = size(r1.state.gridmap)
    for x in 1:extent[1]
        for y in 1:extent[2]
            if r1.state.gridmap[x,y] == -2 && r2.state.gridmap[x,y] != -2
                r1.state.gridmap[x,y] = r2.state.gridmap[x,y]
                r1.state.known_cells += 1
            end
        end
    end
end


function merge_gridmap!(r1::RobotDec, gridmap2::MMatrix)
    extent = size(r1.state.gridmap)
    for x in 1:extent[1]
        for y in 1:extent[2]
            if r1.state.gridmap[x,y] == -2 && gridmap2[x,y] != -2
                r1.state.gridmap[x,y] = gridmap2[x,y]
                r1.state.known_cells += 1
            end
        end
    end
end



function merge_gridmaps!(r1::RobotPosMin, r2::RobotPosMin)
    extent = size(r1.gridmap)
    for x in 1:extent[1]
        for y in 1:extent[2]
            if r1.gridmap[x,y] == -2 && r2.gridmap[x,y] != -2
                r1.gridmap[x,y] = r2.gridmap[x,y]
                r2.gridmap[x,y] = r1.gridmap[x,y]
            end
        end
    end
end
 



# function transitive_communication!(r1::RobotDec, r2::RobotDec)
#     r1.plans[r2.id].state = RobotState(r2.id, r2.pos)
#     r1.rollout_parameters.robots_plans[r2.id].state = deepcopy(r1.plans[r2.id].state)
#     r1.state.robots_states[r2.id] = deepcopy(r1.plans[r2.id].state)
#     r1.plans[r2.id].best_sequences, r1.plans[r2.id].assigned_proba = r2.plans[r2.id].best_sequences, r2.plans[r2.id].assigned_proba

#     r1.plans[r2.id].timestamp = r1.state.step

#     for p in r2.plans
#         if p.state.id != r2.id && p.timestamp > r1.plans[p.state.id].timestamp
#             r1.plans[p.state.id].best_sequences, r1.plans[p.state.id].assigned_proba, r1.plans[p.state.id].timestamp = p.best_sequences, p.assigned_proba, r1.p.timestamp

#             r1.plans[p.state.id].state = RobotState(p.state.id, p.state.pos)
#             r1.rollout_parameters.robots_plans[p.state.id].state = deepcopy(r1.plans[p.state.id].state)
#             r1.state.robots_states[p.state.id] = deepcopy(r1.plans[p.state.id].state)
#         end
#     end
#     merge_gridmaps!(r1,r2)
# end




function exchange_positions!(r1::RobotPosMin, r2::RobotPosMin)
    r1.all_robots_pos[r2.id] = r2.pos
    r2.all_robots_pos[r1.id] = r1.pos
end


function exchange_frontiers!(r1::RobotPosMin, r2::RobotPosMin)
    for f in r1.frontiers
        push!(r2.frontiers, f)
    end
    for f in r2.frontiers
        push!(r1.frontiers, f)
    end
end


function best_sequences_to_buffer(r1::Robot, r2::Robot)
    # r1.plans[r2.id].best_sequences, r1.plans[r2.id].assigned_proba = r2.plans[r2.id].best_sequences, r2.plans[r2.id].assigned_proba
    r1.buffers[r2.id].best_sequences, r1.buffers[r2.id].assigned_proba = r2.plans[r2.id].best_sequences, r2.plans[r2.id].assigned_proba
end


function position_to_buffer(r1::RobotDec, r2::RobotDec)
    # r1.plans[r2.id].state = RobotState(r2.id, r2.pos)
    # r1.rollout_parameters.robots_plans[r2.id].state = deepcopy(r1.plans[r2.id].state)
    # r1.state.robots_states[r2.id] = deepcopy(r1.plans[r2.id].state)
    r1.buffers[r2.id].position = r2.pos
end


function gridmap_to_buffer(r1::RobotDec, r2::RobotDec)
    r1.buffers[r2.id].gridmap = deepcopy(r2.state.gridmap)
end


function frontiers_to_buffer(r1::RobotDec, r2::RobotDec)
    r1.buffers[r2.id].frontiers = r2.frontiers
end


function simple_communication!(r1::RobotDec, r2::RobotDec)
    # best_sequences_to_buffer(r1,r2)
    # position_to_buffer(r1,r2)
    # gridmap_to_buffer(r1,r2)
    # frontiers_to_buffer(r1,r2)
    r1.plans[r2.id].best_sequences, r1.plans[r2.id].assigned_proba = r2.plans[r2.id].best_sequences, r2.plans[r2.id].assigned_proba
    r1.plans[r2.id].state = RobotState(r2.id, r2.pos)
    r1.rollout_parameters.robots_plans[r2.id].state = RobotState(r2.id, r2.pos)
    r1.state.robots_states[r2.id] = RobotState(r2.id, r2.pos)
    merge_gridmap!(r1, r2.state.gridmap)
    r1.frontiers[r2.id] = r2.frontiers[r2.id]
    r1.plans[r2.id].timestamp = r1.state.step
    # r1.buffers[r2.id].empty = false
end


function empty_buffers!(r1::RobotDec)
    for (i, buffer) in enumerate(r1.buffers)
        if !buffer.empty
            r1.plans[i].best_sequences, r1.plans[i].assigned_proba = buffer.best_sequences, buffer.assigned_proba
            r1.plans[i].state = RobotState(i, buffer.position)
            r1.rollout_parameters.robots_plans[i].state = deepcopy(r1.plans[i].state)
            r1.state.robots_states[i] = deepcopy(r1.plans[i].state)
            merge_gridmap!(r1, buffer.gridmap)
            for f in buffer.frontiers
                push!(r1.frontiers, f)
            end 

            buffer = new_buffer() 
            buffer.empty = true
        end
    end
end

function new_buffer()
    # best_sequences = []
    # assigned_proba = []
    # position = (0,0)
    # gridmap = MMatrix{1,1}((0,0))
    # frontiers = Set()
    # empty = true
    best_sequences = nothing
    assigned_proba = nothing
    position = nothing
    gridmap = nothing
    frontiers = nothing
    empty = true
    return Buffer(best_sequences, assigned_proba, position, gridmap, frontiers, empty)
end