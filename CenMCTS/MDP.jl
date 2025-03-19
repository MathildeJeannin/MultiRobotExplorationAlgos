POMDPs.actions(m::RobotMDP) = m.possible_actions


POMDPs.stateindex(m::RobotMDP, s::StateCen) = m.indices[s]
POMDPs.actionindex(m::RobotMDP, a::ActionCen) = m.indices[a]

POMDPs.discount(m::RobotMDP) = m.discount

function POMDPs.transition(m::RobotMDP, s::StateCen, a::ActionCen)
    ImplicitDistribution() do x
        extent = size(s.gridmap)

        next_robots_states = Vector{RobotState}(undef, length(s.robots_states))
        for robot in s.robots_states
            next_robots_states[robot.id] = RobotState(robot.id, robot.pos)
        end
        next_gridmap = deepcopy(s.gridmap)
        next_seen = [0 for i in eachindex(s.robots_states)]

        distribution = SparseCat([-1,0],[0,1.0])

        all_robots_pos = [r.pos for r in next_robots_states]
        for rs in s.robots_states

            action = a.directions_vector[rs.id].direction

            next_pos, obstacle_pos = compute_new_pos(next_gridmap, rs.id, all_robots_pos, 1, action)
            next_robots_states[rs.id] = RobotState(rs.id, next_pos)
            all_robots_pos[rs.id] = next_pos

            _, next_seen[rs.id] = gridmap_update!(next_gridmap, 0, rs.id, all_robots_pos, m.vis_range, [obstacle_pos], model, transition = true, distribution = distribution)
        end      

        sp = StateCen(next_gridmap, next_robots_states, next_seen, s.step+1)
        return sp
    end
end


function POMDPs.reward(m::RobotMDP, s::StateCen, a::ActionCen, sp::StateCen)
    return m.reward_function(m,s,a,sp)
end


function simple_reward(m::RobotMDP, s::StateCen, a::ActionCen, sp::StateCen)
    return (count(x->x==-2, s.gridmap) - count(x->x==-2, sp.gridmap))
end


function all_move_reward(m::RobotMDP, s::StateCen, a::ActionCen, sp::StateCen)
    return (count(x->x==-2, s.gridmap) - count(x->x==-2, sp.gridmap)) + count(x->x!=0, sp.seen)*5
end


function repulsive_reward(m::RobotMDP, s::StateCen, a::ActionCen, sp::StateCen)
    s = 0
    d = 0
    nb_robots = length(s.robots_states)
    for i in 1:nb_robots
        for j in i+1:nb_robots
            d += distance(s.robots_states[i].pos, s.robots_states[j].pos)
        end
    end
    return simple_reward(m,s,a,sp) + d
end



function POMDPs.isterminal(m::RobotMDP, s::StateCen)
    return count(i->i==-2, s.gridmap) == 0 
end


function special_Q(m::RobotMDP, s::StateCen, a::ActionCen)
    next_pos = [rs.pos for rs in s.robots_states]
    for rs in s.robots_states
        next_pos[rs.id], _ = compute_new_pos(s.gridmap, rs.id, next_pos, 1, a.directions_vector[rs.id].direction)
    end
    if any(next_pos .== [rs.pos for rs in s.robots_states])
        return -1000000.0
    else
        return 0.0
    end
end



function special_N(m::RobotMDP, s::StateCen, a::ActionCen)
    next_pos = [rs.pos for rs in s.robots_states]
    for rs in s.robots_states
        next_pos[rs.id], _ = compute_new_pos(s.gridmap, rs.id, next_pos, 1, a.directions_vector[rs.id].direction)
    end
    if any(next_pos .== [rs.pos for rs in s.robots_states])
        return 1000000
    else
        return 0
    end
end