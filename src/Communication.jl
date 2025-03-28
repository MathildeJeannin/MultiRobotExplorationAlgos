function ask_gridmap!(r1::Robot, r2::Robot)
    extent = size(r1.state.gridmap)
    for x in 1:extent[1]
        for y in 1:extent[2]
            if r1.state.gridmap[x,y] == -2
                r1.state.gridmap[x,y] = r2.state.gridmap[x,y]
            end
        end
    end
end

function merge_gridmaps!(r1::Robot, r2::Robot)
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


function merge_all_gridmaps(robots::Vector{Robot})
    robots = [model[i] for i in eachindex(model[1].plans)]
    extent = size(robots[1].state.gridmap)
    full_gridmap = MMatrix{extent[1],extent[2]}(Int64.(-2*ones(Int64, extent)))
    for x in 1:extent[1]
        for y in 1:extent[2]
            for robot in robots
                if robot.state.gridmap[x,y] != -2
                    full_gridmap[x,y] = robot.state.gridmap[x,y]
                end
            end
        end
    end
    return full_gridmap
end
        

function exchange_best_sequences!(r1::Robot, r2::Robot)
    r1.plans[r2.id].best_sequences, r1.plans[r2.id].assigned_proba = r2.plans[r2.id].best_sequences, r2.plans[r2.id].assigned_proba
end


function exchange_positions!(r1::RobotDec, r2::RobotDec)
    r1.plans[r2.id].state = RobotState(r2.id, r2.pos)
    r1.rollout_parameters.robots_plans[r2.id].state = deepcopy(r1.plans[r2.id].state)
    r1.state.robots_states[r2.id] = deepcopy(r1.plans[r2.id].state)
end


function transitive_communication!(r1::RobotDec, r2::RobotDec)
    r1.plans[r2.id].state = RobotState(r2.id, r2.pos)
    r1.rollout_parameters.robots_plans[r2.id].state = deepcopy(r1.plans[r2.id].state)
    r1.state.robots_states[r2.id] = deepcopy(r1.plans[r2.id].state)
    r1.plans[r2.id].best_sequences, r1.plans[r2.id].assigned_proba = r2.plans[r2.id].best_sequences, r2.plans[r2.id].assigned_proba

    r1.plans[r2.id].timestamp = r1.state.step

    for p in r2.plans
        if p.state.id != r2.id && p.timestamp > r1.plans[p.state.id].timestamp
            r1.plans[p.state.id].best_sequences, r1.plans[p.state.id].assigned_proba, r1.plans[p.state.id].timestamp = p.best_sequences, p.assigned_proba, r1.p.timestamp

            r1.plans[p.state.id].state = RobotState(p.state.id, p.state.pos)
            r1.rollout_parameters.robots_plans[p.state.id].state = deepcopy(r1.plans[p.state.id].state)
            r1.state.robots_states[p.state.id] = deepcopy(r1.plans[p.state.id].state)
        end
    end
    merge_gridmaps!(r1,r2)
end


function simple_communication!(r1::RobotDec, r2::RobotDec)
    exchange_best_sequences!(r1,r2)
    exchange_positions!(r1,r2)
    merge_gridmaps!(r1,r2)
    r1.plans[r2.id].timestamp = r1.state.step
end


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