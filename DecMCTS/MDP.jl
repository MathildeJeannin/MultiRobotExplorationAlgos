POMDPs.actions(m::RobotMDP) = m.possible_actions


POMDPs.stateindex(m::RobotMDP, s::State) = m.indices[s]
POMDPs.actionindex(m::RobotMDP, a::Action) = m.indices[a]

POMDPs.discount(m::RobotMDP) = m.discount

function POMDPs.transition(m::RobotMDP, s::State, a::Action)
    # TODO : changement lié à vecteur d'actions -> 1 action + changement de types
    ImplicitDistribution() do x
        robot = model[s.id]
        nb_robots = length(s.robots_states)
        extent = size(s.gridmap)

        # P_obstacle = (m.nb_obstacle - nb_obst_vus)/(extent[1]*extent[2] - nb_cell_vues)
        P_obstacle = 0
        distribution = SparseCat([-1,0], [P_obstacle,1-P_obstacle])

        next_robots_states = deepcopy(s.robots_states)
        next_gridmap = deepcopy(s.gridmap)
        # next_seen_gridmap = deepcopy(s.seen_gridmap)

        if s.nb_coups - robot.rollout_parameters.debut_rollout == 0 
            sequences, states = select_best_sequences(robot)
            for i in eachindex(sequences)
                if i!=robot.id
                    robot.rollout_parameters.robots_plans[i].best_sequences = sequences[i]
                    robot.rollout_parameters.robots_plans[i].state = states[i]
                end
            end
        end

        next_pos, obstacle_pos = compute_new_pos(s.gridmap, robot.id, [rs.pos for rs in next_robots_states], 1, a.direction)

        next_robots_states[robot.id] = RobotState(robot.id, next_pos)

        next_known_cells, next_seen_cells = gridmap_update!(next_gridmap, s.known_cells, robot.id, [rs.pos for rs in next_robots_states], m.vis_range, [obstacle_pos], model, transition = true, distribution = distribution, seen_cells = s.seen_cells)


        for plan in robot.rollout_parameters.robots_plans
            if plan.state.id != robot.id
                if m.use_old_info || (!m.use_old_info && s.nb_coups < plan.timestamp)
                
                    if !isempty(plan.best_sequences) && !isempty(plan.best_sequences[1]) && (length(plan.best_sequences[1]) >= s.nb_coups - robot.rollout_parameters.debut_rollout + 1)
                        action = popfirst!(plan.best_sequences[1])
                        plan.timestamp += 1
                    else
                        action = rand(m.possible_actions)
                    end
                    
                    next_robot_pos, obstacle_pos = compute_new_pos(next_gridmap, plan.state.id, [rs.pos for rs in next_robots_states], 1, action.direction)

                    next_robots_states[plan.state.id] = RobotState(plan.state.id, next_robot_pos)

                    next_known_cells, _ = gridmap_update!(next_gridmap, next_known_cells, plan.state.id, [rs.pos for rs in next_robots_states], m.vis_range, [obstacle_pos], model, transition = true, distribution = distribution)
                end
            end
        end

        sp = State(robot.id, next_robots_states, next_gridmap, next_known_cells, next_seen_cells, s.nb_coups+1)

        return sp
    end
end


function POMDPs.reward(m::RobotMDP, s::State, a::Action, sp::State)
    return m.reward_function(m,s,a,sp)
end


function comm_reward(m::RobotMDP, s::State, a::Action, sp::State)
    robot = model[s.id]
    r = sp.seen_cells - s.seen_cells

    f(x) = 5/(1+exp(x-robot.com_range))
    Q=0

    for i in eachindex(sp.robots_states)
        d = distance(sp.robots_states[robot.id].pos, sp.robots_states[i].pos)
        Q += f(d)
    end
    return r+Q
end


function simple_reward(m::RobotMDP, s::State, a::Action, sp::State)
    return sp.seen_cells - s.seen_cells
end


function POMDPs.isterminal(m::RobotMDP, s::State)
    extent = size(s.gridmap)
    return extent[1]*extent[2]-s.known_cells <= abmproperties(model).invisible_cells[1]
end



struct FrontierPolicy{M<:RobotMDP} <: Policy 
    mdp::M
end


function POMDPs.action(rollout_policy::FrontierPolicy, s::State)
    robot = model[s.id]

    if isempty(robot.rollout_parameters.frontiers)
        return rand(rollout_policy.mdp.possible_actions)
    end

    if robot.rollout_parameters.goal == (0,0) || s.gridmap[robot.rollout_parameters.goal[1], robot.rollout_parameters.goal[2]] != -2
        robot.rollout_parameters.frontiers = frontierDetectionMCTS(s.gridmap, robot.rollout_parameters.frontiers, need_repartition=false)
        robot.rollout_parameters.goal = rand(robot.rollout_parameters.frontiers)
    end

    pos = s.robots_states[s.id].pos
    plan = collect(Agents.Pathfinding.find_path(robot.pathfinder, pos, robot.rollout_parameters.goal))

    if isempty(plan)
        robot.rollout_parameters.goal = (0,0)
        out = rand(rollout_policy.mdp.possible_actions)
        return out
    end

    a = (plan[1][1]-pos[1], plan[1][2]-pos[2])./distance(plan[1], pos)
    return Action(round.(a, digits=2))    
end



function special_Q(m::RobotMDP, s::State, a::Action)
    next_pos, obstacle_pos = compute_new_pos(s.gridmap, s.id, [rs.pos for rs in s.robots_states], 1, a.direction)
    if next_pos == s.robots_states[s.id].pos
        return -1000000.0
    else
        return 0.0
    end
end



function special_N(m::RobotMDP, s::State, a::Action)
    next_pos, obstacle_pos = compute_new_pos(s.gridmap, s.id, [rs.pos for rs in s.robots_states], 1, a.direction)
    if next_pos == s.robots_states[s.id].pos
        return 1000000
    else
        return 0
    end
end