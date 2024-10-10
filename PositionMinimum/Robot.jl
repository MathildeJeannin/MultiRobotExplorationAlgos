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

    global model = AgentBasedModel(Robot{D}, space; agent_step!,
        scheduler = scheduler, 
        properties = properties
    )


    #obstacles
    if num_map == 0
        add_obstacles(model, nb_robots; N = nb_obstacles, extent = extent)
    elseif num_map > 0
        add_map(model, num_map, nb_robots)
    else 
        add_simple_obstacles(model, extent, nb_robots; N = 3)
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

        policy = myPolicy(mdp)

        solver = DPWSolver(n_iterations = n_iterations, depth = depth, max_time = max_time, keep_tree = keep_tree, show_progress = show_progress, enable_action_pw = true, enable_state_pw = true, tree_in_info = true, alpha_state = alpha_state, k_state = k_state, alpha_action = alpha_action, k_action = k_action, exploration_constant = exploration_constant, estimate_value = RolloutEstimator(policy, max_depth=-1))

        planner = solve(solver, mdp)

        agent = Robot{D}(id, pos, vis_range, com_range, isObstacle, [Robot_plan(robot_state(i, (1,1)), Vector{Vector{action_robot}}(undef, 0), Float64[]) for i in 1:nb_robots], Rollout_info(false, 0), state, planner)
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


function compute_new_pos(gridmap::MMatrix, id::Int, robots_pos::Union{Vector, SizedVector}, vis_range::Int, action::action_robot)
    pos = robots_pos[id]
    other_robots_pos = robots_pos[1:end .!= id, :]

    extent = size(gridmap)
    ray = raytracing(pos, (vis_range .* action.action) .+ pos, vis_range)
    x,y = ray[1][1],ray[1][2]

    if length(ray)==1
        return (x,y), (0,0)
    
    else
        for element in ray[2:end]
            x_prev, y_prev = x,y 
            x,y = element[1], element[2]

            if (x > extent[1]) || (y > extent[2]) || (x < 1) || (y < 1)
                return (x_prev, y_prev), (0,0)
            end 

            if gridmap[x,y] == -1 
                return (x_prev, y_prev), (x,y)
            elseif (x,y) in other_robots_pos
                return (x_prev,y_prev), (0,0)
            end 
        end
    end
    return (x,y), (0,0)
end


function agent_step!(robot, model, vis_tree)
    

end