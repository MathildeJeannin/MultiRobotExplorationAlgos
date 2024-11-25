POMDPs.actions(m::RobotMDP) = m.possible_actions


POMDPs.stateindex(m::RobotMDP, s::State) = m.indices[s]
POMDPs.actionindex(m::RobotMDP, a::Action) = m.indices[a]

POMDPs.discount(m::RobotMDP) = m.discount

function POMDPs.transition(m::RobotMDP, s::State, a::Action)
    # TODO : changement lié à vecteur d'actions -> 1 action + changement de types
    ImplicitDistribution() do x
        robot = model[s.id]

        # P_obstacle = (m.nb_obstacle - nb_obst_vus)/(extent[1]*extent[2] - nb_cell_vues)
        P_obstacle = 0
        distribution = SparseCat([-1,0], [P_obstacle,1-P_obstacle])

        next_robots_plans = deepcopy(s.robots_plans)
        next_gridmap = deepcopy(s.gridmap)
        # next_seen_gridmap = deepcopy(s.seen_gridmap)

        if s.nb_coups - robot.rollout_parameters.debut_rollout == 0 
            sequences, states = select_best_sequences(robot)
            for i in eachindex(sequences)
                if i!=robot.id
                    next_robots_plans[i].best_sequences = sequences[i]
                    next_robots_plans[i].state = states[i]
                end
            end
        end

        next_pos, obstacle_pos = compute_new_pos(s.gridmap, robot.id, [p.state.pos for p in next_robots_plans], m.vis_range, a.direction)

        next_robots_plans[robot.id].state = RobotState(robot.id, next_pos)

        next_known_cells, next_seen_cells = gridmap_update!(next_gridmap, s.known_cells, robot.id, [p.state.pos for p in next_robots_plans], m.vis_range, [obstacle_pos], model, transition = true, distribution = distribution, seen_cells = s.seen_cells)

        # if (s.nb_coups-1)%m.frontier_frequency == 0 
        #     robot.rollout_parameters.frontiers = frontierDetection(robot.id, next_pos, m.vis_range, next_gridmap, [p.state.pos for p in next_robots_plans], robot.rollout_parameters.frontiers, need_repartition=false)
        # end


        for plan in next_robots_plans
            if plan.state.id != robot.id
                
                if !isempty(plan.best_sequences) && !isempty(plan.best_sequences[1]) && (length(plan.best_sequences[1]) >= s.nb_coups - robot.rollout_parameters.debut_rollout + 1)
                    action = plan.best_sequences[1][s.nb_coups - robot.rollout_parameters.debut_rollout + 1]
                else
                    action = rand(m.possible_actions)
                end
                
                next_robot_pos, obstacle_pos = compute_new_pos(next_gridmap, plan.state.id, [p.state.pos for p in next_robots_plans], m.vis_range, action.direction)

                next_robots_plans[plan.state.id].state = RobotState(plan.state.id, next_robot_pos)

                next_known_cells, _ = gridmap_update!(next_gridmap, next_known_cells, plan.state.id, [p.state.pos for p in next_robots_plans], m.vis_range, [obstacle_pos], model, transition = true, distribution = distribution)

                # if (s.nb_coups-1)%m.frontier_frequency == 0
                #     robot.rollout_parameters.frontiers = frontierDetection(plan.state.id, next_robot_pos, m.vis_range, next_gridmap, [p.state.pos for p in next_robots_plans], robot.rollout_parameters.frontiers, need_repartition=false)
                # end
            end
        end

        sp = State(robot.id, next_gridmap, next_known_cells, next_seen_cells, next_robots_plans, s.nb_coups+1)

        return sp
    end
end


function POMDPs.reward(m::RobotMDP, s::State, a::Action, sp::State)
    robot = model[s.id]
    r = sp.seen_cells - s.seen_cells

    # a = -4/(robot.com_range - robot.vis_range)^2
    # b = -a*(robot.com_range + robot.vis_range)
    # c = a*(robot.com_range*robot.vis_range)
    # f(x) = a*x^2+b*x+c

    mu = (robot.vis_range+robot.com_range)/2
    sigma = robot.com_range-robot.vis_range
    f(x) = (1/(sigma*sqrt(2*pi)))*exp(-0.5*((x-mu)/sigma)^2)
    Q = 0 
    for i in eachindex(length(sp.robots_plans))
        d = distance(sp.robots_plans[robot.id].state.pos, sp.robots_plans[i].state.pos)
        Q += f(d)
    end
    return r*Q
end


function POMDPs.isterminal(m::RobotMDP, s::State)
    extent = size(s.gridmap)
    return extent[1]*extent[2]-s.known_cells == abmproperties(model).invisible_cells
end



struct FrontierPolicy{M<:RobotMDP} <: Policy 
    mdp::M
end


function POMDPs.action(rollout_policy::FrontierPolicy, s::State)
    robot = model[s.id]

    # if isempty(robot.rollout_parameters.frontiers)
    return rand(rollout_policy.mdp.possible_actions)
    # end

    # if s.nb_coups - robot.rollout_parameters.debut_rollout == 0 
    #     robot.rollout_parameters.frontiers = deepcopy(robot.frontiers)
    #     robot.rollout_parameters.actions_sequence = MutableLinkedList{Action}()
    # end

    # if isempty(robot.rollout_parameters.actions_sequence)
    #     goal = rand(robot.rollout_parameters.frontiers)
    #     pos = s.robots_plans[s.id].state.pos

    #     plan = collect(Agents.Pathfinding.find_path(robot.pathfinder, pos, goal))

    #     if isempty(plan)
    #         return rand(rollout_policy.mdp.possible_actions)
    #     end

    #     robot.rollout_parameters.actions_sequence = MutableLinkedList{Action}()

    #     for p in collect(plan)
    #         action = (p[1]-pos[1], p[2]-pos[2])./distance(p, pos)
    #         all_directions = rollout_policy.mdp.possible_actions
    #         best_direction = all_directions[1]
    #         dist=10000000
    #         for d in all_directions
    #             tmp_dist = distance(action,d.direction)
    #             if tmp_dist < dist
    #                 dist = tmp_dist
    #                 best_direction = d
    #             end
    #         end
    #         append!(robot.rollout_parameters.actions_sequence, best_direction)
    #     end
    # end

    # output = popfirst!(robot.rollout_parameters.actions_sequence)
    
    # return output
end