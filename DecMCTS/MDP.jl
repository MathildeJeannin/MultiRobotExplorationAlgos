POMDPs.actions(m::RobotMDP) = m.possible_actions


POMDPs.stateindex(m::RobotMDP, s::StateDec) = m.indices[s]
POMDPs.actionindex(m::RobotMDP, a::ActionDec) = m.indices[a]

POMDPs.discount(m::RobotMDP) = m.discount

function POMDPs.transition(m::RobotMDP, s::StateDec, a::ActionDec)
    ImplicitDistribution() do x
        robot = model[s.id]
        nb_robots = length(s.robots_states)
        extent = size(s.gridmap)

        P_obstacle = 0
        distribution = SparseCat([-1,0], [P_obstacle,1-P_obstacle])

        next_robots_states = deepcopy(s.robots_states)
        next_gridmap = deepcopy(s.gridmap)

        rollout_parameters = robot.rollout_parameters

        if s.step - rollout_parameters.timestamp_rollout == 0 
            rollout_parameters.in_rollout = false
            sequences, states, timestamps = select_best_sequences(robot)
            for i in eachindex(sequences)
                if i!=robot.id
                    rollout_parameters.robots_plans[i].best_sequences = sequences[i]
                    rollout_parameters.robots_plans[i].state = states[i]
                    rollout_parameters.robots_plans[i].timestamp = timestamps[i]
                end
            end
        end

        if s in robot.planner.tree.s_labels
            rollout_parameters.in_rollout = false
        end

        next_pos, obstacle_pos = compute_new_pos(s.gridmap, robot.id, [rs.pos for rs in next_robots_states], 1, a.direction)

        next_robots_states[robot.id] = RobotState(robot.id, next_pos)

        next_known_cells, next_seen_cells = gridmap_update!(next_gridmap, s.known_cells, robot.id, [rs.pos for rs in next_robots_states], m.vis_range, [obstacle_pos], model, transition = true, distribution = distribution, seen_cells = s.seen_cells)


        for plan in rollout_parameters.robots_plans
            if plan.state.id != robot.id
                if !isempty(plan.best_sequences) && !isempty(plan.best_sequences[1]) && (m.use_old_info || (!m.use_old_info && s.step - plan.timestamp < length(plan.best_sequences[1])))
                    action = popfirst!(plan.best_sequences[1])
                    plan.timestamp = s.step+1
                else
                    action = rand(m.possible_actions)
                end
                
                next_robot_pos, obstacle_pos = compute_new_pos(next_gridmap, plan.state.id, [rs.pos for rs in next_robots_states], 1, action.direction)

                next_robot_pos, obstacle_pos = compute_new_pos(next_gridmap, plan.state.id, [rs.pos for rs in next_robots_states], 1, action.direction)

                next_robots_states[plan.state.id] = RobotState(plan.state.id, next_robot_pos)

                next_known_cells, _ = gridmap_update!(next_gridmap, next_known_cells, plan.state.id, [rs.pos for rs in next_robots_states], m.vis_range, [obstacle_pos], model, transition = true, distribution = distribution)
            end
        end

        sp = StateDec(robot.id, next_robots_states, next_gridmap, next_known_cells, next_seen_cells, s.step+1)
        return sp
    end
end


function POMDPs.reward(m::RobotMDP, s::StateDec, a::ActionDec, sp::StateDec)
    return m.reward_function(m,s,a,sp)
end


function sigmoid_reward(m::RobotMDP, s::StateDec, a::ActionDec, sp::StateDec)
    robot = model[s.id]
    r = sp.seen_cells - s.seen_cells

    f(x) = 1/(1+exp(x-robot.com_range))
    Q=[]

    plans = robot.rollout_parameters.robots_plans
    for state in sp.robots_states
        if state.id != robot.id 
            d = distance(sp.robots_states[robot.id].pos, state.pos)
            push!(Q,f(d)/(length(plans)-1))
        end
    end
    return r+maximum(Q)
end


function gaussian_reward(m::RobotMDP, s::StateDec, a::ActionDec, sp::StateDec)
    robot = model[s.id]
    nb_robots = length(s.robots_states)

    r = sp.seen_cells - s.seen_cells

    mu = (robot.vis_range+robot.com_range)/2
    sigma = (robot.com_range-robot.vis_range)/4
    f(x) = (1/(sigma*sqrt(2*pi)))*exp(-0.5*((x-mu)/sigma)^2)
    Q = []
    for i in eachindex(length(sp.robots_states))
        d = distance(sp.robots_states[robot.id].pos, sp.robots_states[i].pos)
        push!(Q,f(d)/(f(mu)*(length(sp.robots_states)-1)))
    end
    return r+maximum(Q)
end


function simple_reward(m::RobotMDP, s::StateDec, a::ActionDec, sp::StateDec)
    return sp.seen_cells - s.seen_cells
    # return sp.seen_cells - s.seen_cells + 1/model[s.id].rollout_parameters.length_route
end


function POMDPs.isterminal(m::RobotMDP, s::StateDec)
    extent = size(s.gridmap)
    return extent[1]*extent[2]-s.known_cells <= abmproperties(model).invisible_cells[1]
end



function frontier_rollout(m::RobotMDP, s::StateDec, d::Int)
    rollout_parameters = model[s.id].rollout_parameters
    
    if isempty(rollout_parameters.route) || !rollout_parameters.in_rollout
        rollout_parameters.in_rollout = true
        rollout_parameters.route = nouvelle_route(rollout_parameters, s)
    end


    if isempty(rollout_parameters.route) #goal impossible donc path = [pos] et route = [], 
        a = ActionDec((0,0))
    else
        # TODO trouver pourquoi des fois pos = Int au lieu de Tuple{Int,Int}
        if distance(rollout_parameters.route[1].pos, s.robots_states[s.id].pos) > 1 # l'appel à transition précédent n'a pas pu bouger le robot car obstacle ou voisin, il est donc resté immobile
            rollout_parameters.route = nouvelle_route(rollout_parameters, s)
        end

        next_astar_state = popfirst!(rollout_parameters.route)
        next_pos = next_astar_state.pos
        direction = (next_pos .- s.robots_states[s.id].pos)./distance(next_pos, s.robots_states[s.id].pos)

        a = ActionDec((round(direction[1], digits=2), round(direction[2], digits=2)))
    end

    sp, r = @gen(:sp, :r)(m, s, a, model[s.id].planner.rng)

    if d > 0 && !isterminal(m, sp)
        return r + m.discount*frontier_rollout(m, sp, d-1)
    else
        return r
    end
end



function nouvelle_route(rollout_parameters::RolloutInfo, s::StateDec)
    rollout_parameters.frontiers = frontierDetectionMCTS(s.gridmap, rollout_parameters.frontiers, need_repartition=false)
    if isempty(rollout_parameters.frontiers) 
        return 1
    end

    start = AStarState(s.robots_states[s.id].pos, s.gridmap)
    goal = AStarState(rand(rollout_parameters.frontiers), s.gridmap)

    astar_results = astar(astar_neighbours, start, goal)
    route = astar_results.path[2:end]
    return route
end