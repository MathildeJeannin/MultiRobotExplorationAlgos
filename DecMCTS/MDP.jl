POMDPs.actions(m::robotMDP) = m.possible_actions


POMDPs.stateindex(m::robotMDP, s::mdp_state) = m.indices[s]
POMDPs.actionindex(m::robotMDP, a::action_robot) = m.indices[a]

POMDPs.discount(m::robotMDP) = m.discount

function POMDPs.transition(m::robotMDP, s::mdp_state, a::action_robot)
    # TODO : changement lié à vecteur d'actions -> 1 action + changement de types
    ImplicitDistribution() do x
        robot = model[s.space_state.id]

        # P_obstacle = (m.nb_obstacle - nb_obst_vus)/(extent[1]*extent[2] - nb_cell_vues)
        P_obstacle = 0
        distribution = SparseCat([-1,0], [P_obstacle,1-P_obstacle])

        next_robots_plans = deepcopy(s.space_state.robots_plans)
        next_gridmap = deepcopy(s.space_state.gridmap)
        # next_seen_gridmap = deepcopy(s.space_state.seen_gridmap)

        if s.nb_coups - robot.rollout_info.debut_rollout == 0 
            sequences, states = select_best_sequences(robot)
            for i in eachindex(sequences)
                if i!=robot.id
                    next_robots_plans[i].best_sequences = sequences[i]
                    next_robots_plans[i].state = states[i]
                end
            end
        end

        next_pos, obstacle_pos = compute_new_pos(s.space_state.gridmap, robot.id, [p.state.pos for p in next_robots_plans], m.vis_range, a.action)

        next_robots_plans[robot.id].state = robot_state(robot.id, next_pos)

        next_known_cells, next_seen_cells = gridmap_update!(next_gridmap, s.space_state.known_cells, robot.id, [p.state.pos for p in next_robots_plans], m.vis_range, [obstacle_pos], model, transition = true, distribution = distribution, seen_cells = s.space_state.seen_cells)


        for plan in next_robots_plans
            if plan.state.id != robot.id
                
                if !isempty(plan.best_sequences) && !isempty(plan.best_sequences[1]) && (length(plan.best_sequences[1]) >= s.nb_coups - robot.rollout_info.debut_rollout + 1)
                    action = plan.best_sequences[1][s.nb_coups - robot.rollout_info.debut_rollout + 1]
                else
                    action = rand(m.possible_actions)
                end
                
                next_robot_pos, obstacle_pos = compute_new_pos(next_gridmap, plan.state.id, [p.state.pos for p in next_robots_plans], m.vis_range, action.action)

                next_robots_plans[plan.state.id].state = robot_state(plan.state.id, next_robot_pos)

                next_known_cells, _ = gridmap_update!(next_gridmap, next_known_cells, plan.state.id, [p.state.pos for p in next_robots_plans], m.vis_range, [obstacle_pos], model, transition = true, distribution = distribution)

            end
        end

        next_space_state = space_state(robot.id, next_gridmap, next_known_cells, next_seen_cells, next_robots_plans)
        sp = mdp_state(next_space_state, s.nb_coups+1)
        return sp

        _print_gridmap(sp.space_state.gridmap, [])
    end
end


function POMDPs.reward(m::robotMDP, s::mdp_state, a::action_robot, sp::mdp_state)
    robot = model[s.space_state.id]
    r = sp.space_state.seen_cells - s.space_state.seen_cells

    # println(count(x->x!=-2, sp.space_state.gridmap) == sp.space_state.known_cells)

    # global use_penalite
    # if use_penalite
    #     penalite = false
    #     for i in eachindex(length(sp.space_state.robots_plans))
    #         if sqrt((sp.space_state.robots_plans[robot.id].state.pos[1]-sp.space_state.robots_plans[i].state.pos[1])^2 + (sp.space_state.robots_plans[robot.id].state.pos[2]-sp.space_state.robots_plans[i].state.pos[2])^2) > robot.com_range && !penalite
    #             penalite = true
    #             r = r/10
    #         end
    #     end
    # end

    # a = -4/(robot.com_range - robot.vis_range)^2
    # b = -a*(robot.com_range + robot.vis_range)
    # c = a*(robot.com_range*robot.vis_range)
    # f(x) = a*x^2+b*x+c

    mu = (robot.vis_range+robot.com_range)/2
    sigma = robot.com_range-robot.vis_range
    f(x) = (1/(sigma*sqrt(2*pi)))*exp(-0.5*((x-mu)/sigma)^2)
    Q = 0 
    for i in eachindex(length(sp.space_state.robots_plans))
        d = distance(sp.space_state.robots_plans[robot.id].state.pos, sp.space_state.robots_plans[i].state.pos)
        Q += f(d)
    end
    return r*Q
end


function POMDPs.isterminal(m::robotMDP, s::mdp_state)
    extent = size(s.space_state.gridmap)
    return extent[1]*extent[2]-s.space_state.known_cells == abmproperties(model).invisible_cells
end


function _print_state(s::mdp_state)
    println("Robot $(s.space_state.id)")
    println("nb_coups = $(s.nb_coups)")
    println("gridmap = ")
    _print_gridmap(s.space_state.gridmap, [p.state for p in s.space_state.robots_plans])
    println("seen_gridmap = ")
    _print_seen_gridmap(s.space_state.seen_gridmap)
    for p in s.space_state.robots_plans
        println("Robot $(p.state.id), pos = $(p.state.pos)\n")
        println("best sequences")
        i=1
        for seq in p.best_sequences
            println("action $i = $(seq)")
        end
        println("proba = $(p.assigned_proba)\n")
    end
end
