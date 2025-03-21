POMDPs.actions(m::RobotMDP, s::StateDec) = isempty(s.frontiers) ? m.possible_actions : frontiers_to_actions(s.frontiers)

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

        if s.step - robot.rollout_parameters.timestamp_rollout == 0 
            sequences, states, timestamps = select_best_sequences(robot)
            for i in eachindex(sequences)
                if i!=robot.id
                    robot.rollout_parameters.robots_plans[i].best_sequences = sequences[i]
                    robot.rollout_parameters.robots_plans[i].state = states[i]
                    robot.rollout_parameters.robots_plans[i].timestamp = timestamps[i]
                end
            end
        end

        # next_pos, obstacle_pos = compute_new_pos(s.gridmap, robot.id, [rs.pos for rs in next_robots_states], 1, (nothing), goal=a.goal)

        # robot.rollout_parameters.route = plan_route!(s.robots_states[s.id].pos, a.goal, robot.rollout_parameters.pathfinder)
        start = (s.id, [sr.pos for sr in s.robots_states], next_gridmap)
        goal = (s.id, [sr.pos for sr in s.robots_states], next_gridmap)
        goal[2][s.id] = a.goal
        astar_results = astar(astar_neighbours, start, goal)
        robot.rollout_parameters.route = astar_results.path

        next_pos = robot.rollout_parameters.route[end][2][s.id]
        # next_pos = a.goal

        next_robots_states[robot.id] = RobotState(robot.id, next_pos)

        next_known_cells, next_seen_cells = gridmap_update!(next_gridmap, s.known_cells, robot.id, [rs.pos for rs in next_robots_states], m.vis_range, [(0,0)], model, transition = true, distribution = distribution, seen_cells = s.seen_cells)


        for plan in robot.rollout_parameters.robots_plans
            if plan.state.id != robot.id
                if !isempty(plan.best_sequences) && !isempty(plan.best_sequences[1]) && (m.use_old_info || (!m.use_old_info && s.step - plan.timestamp < length(plan.best_sequences[1])))
                    next_robot_pos.goal = popfirst!(plan.best_sequences[1])
                    plan.timestamp = s.step+1
                else
                    next_robot_pos, _ = compute_new_pos(next_gridmap, plan.state.id, [rs.pos for rs in next_robots_states], 1, rand(m.possible_actions).goal, goal=nothing)
                end

                next_robots_states[plan.state.id] = RobotState(plan.state.id, next_robot_pos)

                next_known_cells, _ = gridmap_update!(next_gridmap, next_known_cells, plan.state.id, [rs.pos for rs in next_robots_states], m.vis_range, [(0,0)], model, transition = true, distribution = distribution)
            end
        end

        next_frontiers = deepcopy(s.frontiers)
        # frontierDetectionMCTS!(next_gridmap, next_frontiers, need_repartition = false)
        next_frontiers = frontierDetection(s.id, next_robots_states[s.id].pos, robot.vis_range, next_gridmap, [sr.pos for sr in next_robots_states], next_frontiers; need_repartition=false)


        sp = StateDec(robot.id, next_robots_states, next_gridmap, next_known_cells, next_seen_cells, next_frontiers, s.step+1)
        return sp
    end
end


function POMDPs.reward(m::RobotMDP, s::StateDec, a::ActionDec, sp::StateDec)
    return m.reward_function(m,s,a,sp)
end


function sigmoid_reward(m::RobotMDP, s::StateDec, a::ActionDec, sp::StateDec)
    robot = model[s.id]
    r = simple_reward(m, s, a, sp)

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

    r = simple_reward(m, s, a, sp)

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
    r = sp.seen_cells - s.seen_cells
    N_route = length(model[s.id].rollout_parameters.route)    
    if N_route == 0 
        r_prime = 0
    else
        r_prime = 1/N_route
    end
    return r + r_prime
    # return r
end


function POMDPs.isterminal(m::RobotMDP, s::StateDec)
    extent = size(s.gridmap)
    return extent[1]*extent[2]-s.known_cells <= abmproperties(model).invisible_cells[1]
end


function frontiers_to_actions(frontiers::Set)
    actions = []
    for cell in frontiers
        push!(actions, ActionDec(cell))
    end
    return actions
end