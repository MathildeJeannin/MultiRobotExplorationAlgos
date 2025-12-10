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
    return ((count(x->x==-2, s.gridmap) - count(x->x==-2, sp.gridmap)))/(8*length(s.robots_states))
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


function frontier_rollout(m::RobotMDP, s::StateCen, d::Int)
    rollout_parameters = abmproperties(model).rollout_parameters
    
    nb_robots = length(s.robots_states)
    r = 0
    a = ActionCen([ActionDec((0.0,0.0)) for i in 1:nb_robots])

    computed_frontiers = false
    for (i,_) in enumerate(s.robots_states)
        if (isempty(rollout_parameters.route[i]) || !rollout_parameters.in_rollout) && !computed_frontiers
            rollout_parameters.frontiers = frontierDetectionMCTS(gridmap, rollout_parameters.frontiers, need_repartition=false)
            rollout_parameters.in_rollout = true
            computed_frontiers = true
        end
    end

    for (i,r_state) in enumerate(s.robots_states)

        if  isempty(rollout_parameters.route[i]) || !rollout_parameters.in_rollout
            rollout_parameters.route[i] = nouvelle_route(rollout_parameters, r_state.pos, s.gridmap)
            if isempty(rollout_parameters.route[i])
                a.directions_vector[i] = ActionDec((0.0,0.0))
                break
            end
        end

        if distance(rollout_parameters.route[i][1].pos, r_state.pos) > 1 || distance(rollout_parameters.route[i][1].pos, r_state.pos) == 0 # l'appel à transition précédent n'a pas pu bouger le robot car obstacle ou voisin, il est donc resté immobile ou probleme avec 1ere action
            rollout_parameters.route[i] = nouvelle_route(rollout_parameters, r_state.pos, gridmap)
            if isempty(rollout_parameters.route[i])
                a.directions_vector[i] = ActionDec((0.0,0.0))
                break
            end
        end

        
        next_astar_state = popfirst!(rollout_parameters.route[i])

        next_pos = next_astar_state.pos
        direction = (next_pos .- r_state.pos)./distance(next_pos, r_state.pos)
        a.directions_vector[i] = ActionDec((round(direction[1], digits=2), round(direction[2], digits=2)))
    end

    sp, r = @gen(:sp, :r)(m, s, a, planner.rng)

    if d > 0 && !isterminal(m, sp)
        return r + m.discount*frontier_rollout(m, sp, d-1)
    else
        return r
    end
end



function nouvelle_route(rollout_parameters::RolloutInfo, pos::Tuple, gridmap::MMatrix)
    if isempty(rollout_parameters.frontiers) 
        return []
    end

    #TODO : a enlever apres test sur carte connue
    # s_carte_connue = deepcopy(s)
    # add_walls_to_gridmap!(s_carte_connue.gridmap, abmproperties(model).num_map)
    # ##

    start = AStarState(pos, gridmap)
    # goal_cell = goToFrontier(rand(rollout_parameters.frontiers), s.robots_states[s.id].pos, s.gridmap)
    goal_cell = rand(rollout_parameters.frontiers)
    goal = AStarState(goal_cell, gridmap)

    astar_results = astar(astar_neighbours, start, goal)
    route = astar_results.path[2:end]
    return route
end


function AStarDistance(gridmap::MMatrix, pos1::Tuple, pos2::Tuple)
    start = AStarState(pos1, gridmap)
    goal = AStarState(pos2, gridmap)
    astar_results = astar(astar_neighbours, start, goal)
    return length(astar_results.path)
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