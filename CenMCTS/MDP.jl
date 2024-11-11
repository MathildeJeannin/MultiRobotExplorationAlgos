POMDPs.actions(m::RobotMDP) = m.possible_actions


POMDPs.stateindex(m::RobotMDP, s::StateCen) = m.indices[s]
POMDPs.actionindex(m::RobotMDP, a::ActionCen) = m.indices[a]

POMDPs.discount(m::RobotMDP) = m.discount

function POMDPs.transition(m::RobotMDP, s::StateCen, a::ActionCen)
    ImplicitDistribution() do x
        extent = size(s.gridmap)

        nb_obst_vus = count(x->x==-1, s.gridmap)
        nb_cell_vues = count(x->x!=-2, s.gridmap)

        next_robots_states = Vector{RobotState}(undef, length(s.robots_states))
        for robot in s.robots_states
            next_robots_states[robot.id] = RobotState(robot.id, robot.pos)
        end
        next_gridmap = deepcopy(s.gridmap)
        # next_seen_gridmap = deepcopy(s.seen_gridmap)

        distribution = SparseCat([-1,0],[0,1.0])

        # for i in eachindex(s.robots_states)
        all_robots_pos = [r.pos for r in next_robots_states]
        for rs in s.robots_states

            action = a.directions[rs.id].direction

            next_pos, obstacle_pos = compute_new_pos(next_gridmap, rs.id, all_robots_pos, m.vis_range, action)
            next_robots_states[rs.id] = RobotState(rs.id, next_pos)
            all_robots_pos[rs.id] = next_pos

            gridmap_update!(next_gridmap, 0, rs.id, all_robots_pos, m.vis_range, [obstacle_pos], model, transition = true, distribution = distribution)

        end      

        # println(next_seen_gridmap)

        sp = StateCen(next_gridmap, next_robots_states, s.nb_coups+1)
        return sp
    end
end


function POMDPs.reward(m::RobotMDP, s::StateCen, a::ActionCen, sp::StateCen)
    nb_robots = length(s.robots_states)

    r = (count(x->x==-2, s.gridmap) - count(x->x==-2, sp.gridmap))
    
    return r
end


function POMDPs.isterminal(m::RobotMDP, s::StateCen)
    return count(i->i==-2, s.gridmap) == 0 
end


function compute_actions(nb_robots)
    theta = [i*pi/4 for i in 0:7]
    rad_actions = [(round(cos(θ),digits=2),round(sin(θ), digits=2)) for θ in theta]

    ids = collect(Int8(i) for i in 1:nb_robots)
    
    actionsNR = [Vector{Action}(undef,length(rad_actions)) for i in 1:nb_robots]

    for id in ids
        for index_a in eachindex(rad_actions)
            actionsNR[id][index_a] = Action(rad_actions[index_a])
        end
    end

    A = [actionsNR[i] for i in 1:nb_robots]
    all_actions = collect(Iterators.product(A...))


    global vector_all_actions = Vector{ActionCen}(undef, length(all_actions))
    for i in eachindex(all_actions)
        # vector_all_actions[i] = [j for j in all_actions[i]]
        vector_all_actions[i] = ActionCen([j for j in all_actions[i]])
    end
    
    return vector_all_actions
end
