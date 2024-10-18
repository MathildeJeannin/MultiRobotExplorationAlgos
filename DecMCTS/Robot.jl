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
    invisible_cells = 0
)
    gridmap = MMatrix{extent[1],extent[2]}(Int8.(-2*ones(Int8, extent)))
    # seen_gridmap = MMatrix{extent[1],extent[2]}(Int8.(zeros(Int8, extent)))

    # initialize model
    space = GridSpace(extent, periodic = false, metric = :euclidean)
    scheduler = Schedulers.fastest  # they are executed semi-synchronously,
                # in order of their indexing

    D = length(extent)

    T1 = Tuple(i for i in 1:begin_zone[1])
    T2 = Tuple(i for i in 1:begin_zone[2])

    properties = (
        seen_all_gridmap = BitArray{3}(falses((extent[1],extent[2],nb_robots))),
        nb_obstacles, 
        invisible_cells
    )

    global model = AgentBasedModel(Union{RobotDec{D}, Obstacle{D}}, space; agent_step!,
        scheduler = scheduler, 
        properties = properties
    )


    #obstacles
    if num_map == 0
        add_obstacles(model, nb_robots; N = nb_obstacles, extent = extent)
    elseif num_map > 0
        add_map(model, num_map, nb_robots)
    else 
        abmproperties(model).invisible_cells[1] = add_simple_obstacles(model, extent, nb_robots; N = 3)
    end

    theta = [i*pi/4 for i in 0:7]
    # theta = [i*pi/2 for i in 0:3]
    rad_actions = [(round(cos(θ),digits=2),round(sin(θ), digits=2)) for θ in theta]
    possible_actions = Vector{action_robot}(undef,length(rad_actions))
    for (i,a) in enumerate(rad_actions)
        possible_actions[i] = action_robot(a)
    end

    robots_plans = MVector{nb_robots, Robot_plan}([Robot_plan(robot_state(i,(1,1)), Vector{Vector{Tuple{Int,action_robot}}}(undef, 0), Vector{Float64}(undef, 0)) for i in 1:nb_robots])

    for n ∈ 1:nb_robots
        pos = (rand(T1),rand(T2))
        id = n
        isObstacle = false

        obstacles_poses = [element.pos for element in nearby_obstacles(pos, model, vis_range)]
    
        # TODO : mutable robot_state ??
        robots_plans[id].state = robot_state(id, pos)

        gridmap_n = copy(gridmap)
        # seen_gridmap_n = copy(seen_gridmap)

        known_cells, seen_cells = gridmap_update!(gridmap_n, 0, id, [p.state.pos for p in robots_plans], vis_range, obstacles_poses, model; seen_cells = 0)

        state = mdp_state(space_state(id, gridmap_n, known_cells, seen_cells, deepcopy(robots_plans)), 0)

        mdp = robotMDP(vis_range, nb_obstacles, discount, possible_actions)

        solver = DPWSolver(n_iterations = n_iterations, depth = depth, max_time = max_time, keep_tree = keep_tree, show_progress = show_progress, enable_action_pw = true, enable_state_pw = true, tree_in_info = true, alpha_state = alpha_state, k_state = k_state, alpha_action = alpha_action, k_action = k_action, exploration_constant = exploration_constant, estimate_value = RolloutEstimator(RandomSolver(), max_depth=-1))

        planner = solve(solver, mdp)

        agent = RobotDec{D}(id, pos, vis_range, com_range, [Robot_plan(robot_state(i, (1,1)), Vector{Vector{action_robot}}(undef, 0), Float64[]) for i in 1:nb_robots], Rollout_info(false, 0), state, planner)
        add_agent!(agent, pos, model)
    end

    robots = [model[i] for i in 1:nb_robots]
    for robot in robots
        in_range = nearby_robots(robot, model, robot.com_range)
        for r in in_range
            exchange_positions!(robot, r)
        end
    end

    return model
end


# function compute_new_pos(gridmap::MMatrix, id::Int, robots_pos::Union{Vector, SizedVector}, vis_range::Int, action::action_robot)
#     pos = robots_pos[id]
#     other_robots_pos = robots_pos[1:end .!= id, :]

#     extent = size(gridmap)
#     ray = raytracing(pos, (vis_range .* action.action) .+ pos, vis_range)
#     x,y = ray[1][1],ray[1][2]

#     if length(ray)==1
#         return (x,y), (0,0)
    
#     else
#         for element in ray[2:end]
#             x_prev, y_prev = x,y 
#             x,y = element[1], element[2]

#             # hors zone 
#             if (x > extent[1]) || (y > extent[2]) || (x < 1) || (y < 1)
#                 return (x_prev, y_prev), (0,0)
#             end 

#             # if gridmap[x,y] == -2
#             #     println("walking in the unknown")
#             # end

#             # obstacle/robots, tous les cas
#             if gridmap[x,y] == -1 
#                 return (x_prev, y_prev), (x,y)
#             elseif (x,y) in other_robots_pos
#                 return (x_prev,y_prev), (0,0)
#             end 
#         end
#     end
#     return (x,y), (0,0)
# end


function agents_simulate!(robot, model, alpha, beta;
    nb_sequence = 3,
    fct_proba = compute_q,
    fct_sequence = state_best_average_action, 
    nb_communication = 1
    )
    extent = size(model[1].state.space_state.gridmap)

    if robot.state.space_state.known_cells != extent[1]*extent[2] - abmproperties(model).invisible_cells[1]

        for i in 1:nb_communication

            in_range = nearby_robots(robot, model, robot.com_range)
            for r in in_range
                exchange_positions!(robot, r)
                merge_gridmaps!(robot,r)
                exchange_best_sequences!(robot, r)
            end

            robot.rollout_info.debut_rollout = robot.state.nb_coups
            robot.rollout_info.in_rollout = true
            try 
                action_info(robot.planner, robot.state)
            catch e
            end
            robot.rollout_info.in_rollout = false

            robot.plans[robot.id].best_sequences, robot.plans[robot.id].assigned_proba = select_sequences(robot, nb_sequence, false, fct_proba, fct_sequence)

            update_distribution!(robot, alpha, beta)

        end
    end

end


function agent_step!(robot, model, vis_tree)
    extent = size(model[1].state.space_state.gridmap)
    nb_robots = length(robot.plans)

    if robot.state.space_state.known_cells != extent[1]*extent[2] - abmproperties(model).invisible_cells[1]

        if !isempty(robot.plans[robot.id].best_sequences)
            a = robot.plans[robot.id].best_sequences[findall(item->item==maximum(robot.plans[robot.id].assigned_proba), robot.plans[robot.id].assigned_proba)[1]][1]
        else
            a = rand(robot.planner.mdp.possible_actions)
        end
        
        neighbours = collect(nearby_robots(robot, model, robot.vis_range))
        robots_pos = repeat([(0,0)], outer = nb_robots)
        robots_pos[robot.id] = robot.pos
        for n in neighbours
            robots_pos[n.id] = n.pos
        end

        new_pos,_ = compute_new_pos(robot.state.space_state.gridmap, robot.id, robots_pos, robot.vis_range, a.action)
        move_agent!(robot, new_pos, model)

        robot.state.space_state.robots_plans[robot.id].state = robot_state(robot.id, robot.pos)
        robot.plans[robot.id].state = robot_state(robot.id, robot.pos)
        robots_pos[robot.id] = robot.pos

        obstacles_pos = [element.pos for element in nearby_obstacles(robot, model, robot.vis_range)]

        robot.state = mdp_state(space_state(robot.id, robot.state.space_state.gridmap, robot.state.space_state.known_cells, robot.state.space_state.seen_cells, robot.state.space_state.robots_plans), robot.state.nb_coups+1)

        robot.state.space_state.known_cells, robot.state.space_state.seen_cells = gridmap_update!(robot.state.space_state.gridmap, robot.state.space_state.known_cells, robot.id, robots_pos, robot.vis_range, obstacles_pos, model, seen_cells = robot.state.space_state.seen_cells)

        # if robot.state.nb_coups > 15
	# robot.state.space_state.known_cells += check_for_invisible_obstacles!(robot.state.space_state.gridmap)
        # end

        if vis_tree
            inchrome(D3Tree(robot.planner.tree, init_expand=4))
        end
    end

    # robot.state.space_state.known_cells += check_for_invisible_obstacles!(robot.state.space_state.gridmap)

end
