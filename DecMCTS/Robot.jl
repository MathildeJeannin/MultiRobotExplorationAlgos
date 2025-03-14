MCTS.node_tag(s::GWPos) = "[$(s[1]),$(s[2])]"
MCTS.node_tag(a::Symbol) = "[$a]"

function initialize_model(
    nb_robots,
    extent,
    nb_obstacles, 
    num_map,
    alpha_state, 
    k_state,
    alpha_action,
    k_action,
    exploration_constant, 
    n_iterations, 
    keep_tree, 
    discount, 
    vis_figure,
    vis_tree,
    depth,
    max_time, 
    show_progress;
    begin_zone = (1,1), 
    vis_range = 3.0,
    com_range = 2.0,
    invisible_cells = [0],
    nb_blocs = 0,
    fct_reward = simple_reward,
    use_old_info=true
    )

    gridmap = MMatrix{extent[1],extent[2]}(Int64.(-2*ones(Int64, extent)))
    # seen_gridmap = MMatrix{extent[1],extent[2]}(Int64.(zeros(Int64, extent)))

    # initialize model
    space = GridSpace(extent, periodic = false, metric = :euclidean)
    scheduler = Schedulers.fastest  # they are executed semi-synchronously,
                # in order of their indexing

    D = length(extent)

    T1 = Tuple(i for i in 1:begin_zone[1])
    T2 = Tuple(i for i in 1:begin_zone[2])

    properties = (
        seen_all_gridmap = MVector{nb_robots, MMatrix}(MMatrix{extent[1],extent[2]}(Int64.(zeros(Int64, extent))) for i in 1:nb_robots),
        nb_obstacles, 
        invisible_cells,
        extent,
        max_cell_in_scan = [0],
        nb_robots
    )

    global model = AgentBasedModel(Union{RobotDec{D}, Obstacle{D}}, space; agent_step!,
        scheduler = scheduler, 
        properties = properties
    )

    #abmproperties(model).max_cell_in_scan[1] = maxCellInScan(vis_range, model)

    #obstacles
    if num_map == 0
        add_obstacles(model, nb_robots; N = nb_obstacles[1], extent = extent)
    elseif num_map > 0
        add_map(model, num_map, nb_robots)
    else 
        abmproperties(model).invisible_cells[1], abmproperties(model).nb_obstacles[1] = add_simple_obstacles(model, extent, nb_robots; N = nb_blocs)
    end

    possible_actions = compute_actions_decMCTS()

    robots_plans = MVector{nb_robots, RobotPlan}([RobotPlan(RobotState(i,(1,1)), Vector{Vector{Tuple{Int,Action}}}(undef, 0), Vector{Float64}(undef, 0), 0) for i in 1:nb_robots])

    pos = Vector{Tuple{Int,Int}}(undef, nb_robots)
    for n ∈ 1:nb_robots
        pos[n] = (rand(T1),rand(T2))
        while !isempty(ids_in_position(pos[n], model))
            pos[n] = (rand(T1),rand(T2))
        end
    end

    depth = maximum([depth,maximum(extent)*5])

    for n in 1:nb_robots
        id = n
        isObstacle = false

        obstacles_poses = [element.pos for element in nearby_obstacles(pos[n], model, vis_range)]
    
        robots_plans[id].state = RobotState(id, pos[n])

        gridmap_n = copy(gridmap)
        # seen_gridmap_n = copy(seen_gridmap)

        known_cells, seen_cells = gridmap_update!(gridmap_n, 0, id, [p.state.pos for p in robots_plans], vis_range, obstacles_poses, model; seen_cells = 0)

        # state = State(id, gridmap_n, known_cells, seen_cells, deepcopy(robots_plans), 0)
        robots_states = MVector{nb_robots, RobotState}([robots_plans[i].state for i in 1:nb_robots])
        state = State(id, robots_states, gridmap_n, known_cells, seen_cells, 0)

        mdp = RobotMDP(vis_range, nb_obstacles[1], discount, possible_actions, fct_reward, use_old_info)

        solver = DPWSolver(n_iterations = n_iterations, depth = depth, max_time = max_time, keep_tree = keep_tree, show_progress = show_progress, enable_action_pw = true, enable_state_pw = false, tree_in_info = true, alpha_state = alpha_state, k_state = k_state, alpha_action = alpha_action, k_action = k_action, exploration_constant = exploration_constant, estimate_value = RolloutEstimator(RandomSolver(), max_depth=-1))
        
        # my_policy = FrontierPolicy(mdp)
        # solver = DPWSolver(n_iterations = n_iterations, depth = depth, max_time = max_time, keep_tree = keep_tree, show_progress = show_progress, enable_action_pw = true, enable_state_pw = false, tree_in_info = true, alpha_state = alpha_state, k_state = k_state, alpha_action = alpha_action, k_action = k_action, exploration_constant = exploration_constant, estimate_value = RolloutEstimator(my_policy), init_Q=special_Q)

        planner = solve(solver, mdp)

        walkmap = BitArray{2}(trues(extent[1],extent[2]))
        pathfinder = Agents.Pathfinding.AStar(abmspace(model), walkmap=walkmap)

        agent = RobotDec{D}(id, pos[n], vis_range, com_range, [RobotPlan(RobotState(i, (1,1)), Vector{MutableLinkedList{Action}}(undef, 0), Float64[], 0) for i in 1:nb_robots], RolloutInfo(false, 0, Set(), (0,0), deepcopy(robots_plans)), state, planner, pathfinder, Set(), 0)
        add_agent!(agent, pos[n], model)
    end

    robots = [model[i] for i in 1:nb_robots]
    for robot in robots
        in_range = nearby_robots(robot, model, robot.com_range)
        for r in in_range
            exchange_positions!(robot, r)
        end

        # robot.rollout_parameters.frontiers = robot.frontiers = frontierDetection(robot.id, robot.pos, robot.vis_range, robot.state.gridmap, [p.state.pos for p in robot.plans], robot.rollout_parameters.frontiers, need_repartition=false)
    end

    return model
end



function agents_simulate!(robot, model, alpha, beta;
    nb_sequence = 3,
    fct_proba = compute_q,
    fct_sequence = state_best_average_action, 
    fr_communication = 1,
    fct_communication = simple_communication!
    )
    extent = size(model[1].state.gridmap)

    if robot.state.known_cells != extent[1]*extent[2] - abmproperties(model).invisible_cells[1]

        function bloc_mcts()
            robot.rollout_parameters.debut_rollout = robot.state.nb_coups
            robot.rollout_parameters.in_rollout = true
            try 
                action_info(robot.planner, robot.state)
            catch e
            end
            robot.rollout_parameters.in_rollout = false

            robot.plans[robot.id].best_sequences, robot.plans[robot.id].assigned_proba = select_sequences(robot, nb_sequence, false, fct_proba, fct_sequence)

            update_distribution!(robot, alpha, beta)
        end

        if fr_communication >= 1
            for i in 1:fr_communication
                in_range = nearby_robots(robot, model, robot.com_range)
                for r in in_range
                    fct_communication(robot,r)
                end
                bloc_mcts()
            end
        else
            in_range = nearby_robots(robot, model, robot.com_range)
            for r in in_range
                if robot.state.nb_coups - robot.plans[r.id].timestamp >= 1/fr_communication
                    fct_communication(robot,r)
                end
            end
            bloc_mcts()
        end
    end
end


function agent_step!(robot, model, vis_tree)
    extent = size(model[1].state.gridmap)
    nb_robots = length(robot.plans)

    if robot.state.known_cells != extent[1]*extent[2] - abmproperties(model).invisible_cells[1]

        if !isempty(robot.plans[robot.id].best_sequences)
            a = first(robot.plans[robot.id].best_sequences[findall(item->item==maximum(robot.plans[robot.id].assigned_proba), robot.plans[robot.id].assigned_proba)[1]])
        else
            a = rand(robot.planner.mdp.possible_actions)
        end
        
        neighbours = collect(nearby_robots(robot, model, robot.vis_range))
        robots_pos = repeat([(0,0)], outer = nb_robots)
        robots_pos[robot.id] = robot.pos
        for n in neighbours
            robots_pos[n.id] = n.pos
        end

        new_pos,_ = compute_new_pos(robot.state.gridmap, robot.id, robots_pos, 1, a.direction)
        move_agent!(robot, new_pos, model)

        robot.state = State(robot.id, deepcopy(robot.state.robots_states), deepcopy(robot.state.gridmap), robot.state.known_cells, robot.state.seen_cells, robot.state.nb_coups+1)

        robot.state.robots_states[robot.id] = RobotState(robot.id, robot.pos)
        robot.plans[robot.id].state = RobotState(robot.id, robot.pos)
        robots_pos[robot.id] = robot.pos

        obstacles_pos = [element.pos for element in nearby_obstacles(robot, model, robot.vis_range)]

        robot.state.known_cells, robot.state.seen_cells = gridmap_update!(robot.state.gridmap, robot.state.known_cells, robot.id, robots_pos, robot.vis_range, obstacles_pos, model, seen_cells = robot.state.seen_cells)

        # robot.rollout_parameters.frontiers = robot.frontiers = frontierDetection(robot.id, robot.pos, robot.vis_range, robot.state.gridmap, [p.state.pos for p in robot.plans], robot.rollout_parameters.frontiers, need_repartition=false)

        pathfinder_update!(robot.pathfinder, robot.state.gridmap)

        if vis_tree
            inchrome(D3Tree(robot.planner.tree, init_expand=4))
        end
    end
end
