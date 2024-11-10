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

        if s.nb_coups - robot.rollout_info.debut_rollout == 0 
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

        next_frontiers = deepcopy(s.frontiers)
        next_frontiers = frontierDetection(robot.id, next_pos, m.vis_range, next_gridmap, [p.state.pos for p in next_robots_plans], next_frontiers, need_repartition=false)


        for plan in next_robots_plans
            if plan.state.id != robot.id
                
                if !isempty(plan.best_sequences) && !isempty(plan.best_sequences[1]) && (length(plan.best_sequences[1]) >= s.nb_coups - robot.rollout_info.debut_rollout + 1)
                    action = plan.best_sequences[1][s.nb_coups - robot.rollout_info.debut_rollout + 1]
                else
                    action = rand(m.possible_actions)
                end
                
                next_robot_pos, obstacle_pos = compute_new_pos(next_gridmap, plan.state.id, [p.state.pos for p in next_robots_plans], m.vis_range, action.direction)

                next_robots_plans[plan.state.id].state = RobotState(plan.state.id, next_robot_pos)

                next_known_cells, _ = gridmap_update!(next_gridmap, next_known_cells, plan.state.id, [p.state.pos for p in next_robots_plans], m.vis_range, [obstacle_pos], model, transition = true, distribution = distribution)

                next_frontiers = frontierDetection(plan.state.id, next_robot_pos, m.vis_range, next_gridmap, [p.state.pos for p in next_robots_plans], next_frontiers, need_repartition=false)
            end
        end

        # next_space_state = space_state(robot.id, next_gridmap, next_known_cells, next_seen_cells, next_robots_plans)
        sp = State(robot.id, next_gridmap, next_known_cells, next_seen_cells, next_robots_plans, next_frontiers, s.nb_coups+1)

        pathfinder_update!(robot.pathfinder, sp.gridmap)
        return sp
    end
end


function POMDPs.reward(m::RobotMDP, s::State, a::Action, sp::State)
    robot = model[s.id]
    r = sp.seen_cells - s.seen_cells

    # println(count(x->x!=-2, sp.gridmap) == sp.known_cells)

    # global use_penalite
    # if use_penalite
    #     penalite = false
    #     for i in eachindex(length(sp.robots_plans))
    #         if sqrt((sp.robots_plans[robot.id].state.pos[1]-sp.robots_plans[i].state.pos[1])^2 + (sp.robots_plans[robot.id].state.pos[2]-sp.robots_plans[i].state.pos[2])^2) > robot.com_range && !penalite
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


function _print_state(s::State)
    println("Robot $(s.id)")
    println("nb_coups = $(s.nb_coups)")
    println("gridmap = ")
    _print_gridmap(s.gridmap, [p.state for p in s.robots_plans])
    println("seen_gridmap = ")
    _print_seen_gridmap(s.seen_gridmap)
    for p in s.robots_plans
        println("Robot $(p.state.id), pos = $(p.state.pos)\n")
        println("best sequences")
        i=1
        for seq in p.best_sequences
            println("action $i = $(seq)")
        end
        println("proba = $(p.assigned_proba)\n")
    end
end


struct FrontierPolicy{M<:RobotMDP} <: Policy 
    mdp::M
end

function POMDPs.action(rollout_policy::FrontierPolicy, s::State)
    goal = rand(s.frontiers)
    pos = s.robots_plans[s.id].state.pos
    plan = collect(plan_route!(pos, goal, model[s.id].pathfinder))
    action = (plan[1][1]-pos[1], plan[1][2]-pos[2])./distance(plan[1], pos)
    
    all_directions = rollout_policy.mdp.possible_actions
    best_direction = all_directions[1]
    dist=10000000
    for d in all_directions
        tmp_dist = distance(action,d)
        if tmp_dist < dist
            dist = tmp_dist
            best_direction = d
        end
    end
    return best_direction
end