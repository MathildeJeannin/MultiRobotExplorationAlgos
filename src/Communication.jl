function ask_gridmap!(r1::Robot, r2::Robot)
    extent = size(r1.state.space_state.gridmap)
    for x in 1:extent[1]
        for y in 1:extent[2]
            if r1.state.space_state.gridmap[x,y] == -2
                r1.state.space_state.gridmap[x,y] = r2.state.space_state.gridmap[x,y]
            end
        end
    end
end

function merge_gridmaps!(r1::RobotDec, r2::RobotDec)
    extent = size(r1.state.space_state.gridmap)
    for x in 1:extent[1]
        for y in 1:extent[2]
            if r1.state.space_state.gridmap[x,y] == -2 && r2.state.space_state.gridmap[x,y] != -2
                r1.state.space_state.gridmap[x,y] = r2.state.space_state.gridmap[x,y]
                r1.state.space_state.known_cells += 1
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
    extent = size(robots[1].state.space_state.gridmap)
    full_gridmap = MMatrix{extent[1],extent[2]}(Int8.(-2*ones(Int8, extent)))
    for x in 1:extent[1]
        for y in 1:extent[2]
            for robot in robots
                if robot.state.space_state.gridmap[x,y] != -2
                    full_gridmap[x,y] = robot.state.space_state.gridmap[x,y]
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
    r1.plans[r2.id].state = robot_state(r2.id, r2.pos)
    r1.state.space_state.robots_plans[r2.id].state = robot_state(r2.id, r2.pos)
end

function exchange_positions!(r1::RobotPosMin, r2::RobotPosMin)
    r1.all_robots_pos[r2.id] = r2.pos
    r2.all_robots_pos[r1.id] = r1.pos
end