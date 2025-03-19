MCTS.node_tag(s::GWPos) = "[$(s[1]),$(s[2])]"
MCTS.node_tag(a::Symbol) = "[$a]"


function initialize_model(;
    nb_robots = 3,               # number of agents
    extent = (20,20),    # size of the world
    begin_zone = (5,5),   # beinning zone for robots
    vis_range = 3.0,       # visibility range
    com_range = 10.0,       # communication range
    nb_obstacles = 0,
    invisible_cells = [0],
    seed = 1,               # random seed
    num_map = 0,
    discount = 0.85,
    n_iterations = 2500,
    depth = 100,
    alpha_state = 1.0, 
    k_state = 500.0, 
    alpha_action = 1.0,
    k_action = 1.0,
    exploration_constant = 1.0,
    keep_tree = false,
    max_time = 60.0,
    show_progress = false,
    max_steps = 500,
    nb_blocs = 0,
    reward_function = all_move_reward
)

    # initialize model
    space = GridSpace(extent, periodic = false, metric = :euclidean)
    rng = Random.MersenneTwister(seed)
    scheduler = Schedulers.fastest

    D = length(extent)

    global gridmap = MMatrix{extent[1],extent[2]}(Int64.(-2*ones(Int64, extent)))
    seen_gridmap = MMatrix{extent[1],extent[2]}(Int64.(zeros(Int64, extent)))

    T1 = Tuple(i for i in 1:begin_zone[1])
    T2 = Tuple(i for i in 1:begin_zone[2])

    frontiers = Set()
    actions_sequence = Vector{ActionCen}(undef, 0)
    #TODO enlever frontieres de rollout info
    rollout_parameters = RolloutInfo(1, MVector{0,Nothing}())

    properties = (
        seen_all_gridmap = MVector{nb_robots, MMatrix}(MMatrix{extent[1],extent[2]}(Int64.(zeros(Int64, extent))) for i in 1:nb_robots),
        nb_obstacles, 
        invisible_cells,
        nb_robots,
        rollout_parameters
    )

    global model = AgentBasedModel(Union{Robot, Obstacle}, space; agent_step!,
        rng = rng,
        scheduler = scheduler,
        properties = properties
    )

    if num_map > 0
        add_map(model, num_map, nb_robots)
    elseif num_map < 0
        abmproperties(model).invisible_cells[1], abmproperties(model).nb_obstacles[1] = add_simple_obstacles(model, extent, nb_robots; N = nb_blocs)
    else
        add_obstacles(model, nb_robots; N = nb_obstacles[1], extent = extent)
    end

    robots_states = Vector{RobotState}(undef, nb_robots)

    pos = Vector{Tuple{Int,Int}}(undef, nb_robots)
    for n in 1:nb_robots
        pos[n] = (rand(T1),rand(T2))
        while !isempty(ids_in_position(pos[n], model))
            pos[n] = (rand(T1),rand(T2))
        end
        robot = RobotCen{D}(n, pos[n], vis_range)
        add_agent!(robot, pos[n], model)
        robots_states[n] = RobotState(robot.id, robot.pos)
    end

    robots = [model[i] for i in 1:nb_robots]
    all_robots_pos = [r.pos for r in robots]
    for robot in robots
        obstacles = nearby_obstacles(robot, model, robot.vis_range)
        obstacles_pos = [element.pos for element in obstacles]
        gridmap_update!(gridmap, 0, robot.id, all_robots_pos, robot.vis_range, obstacles_pos, model)
    end

    possible_actions = compute_actions_cenMCTS(nb_robots)

    mdp = RobotMDP(vis_range, nb_obstacles[1], discount, possible_actions, reward_function, true)

    depth = maximum([depth,maximum(extent)*5])

    solver = DPWSolver(n_iterations = n_iterations, depth = depth, max_time = max_time, keep_tree = keep_tree, show_progress = show_progress, enable_action_pw = true, enable_state_pw = false, tree_in_info = true, alpha_state = alpha_state, k_state = k_state, alpha_action = alpha_action, k_action = k_action, exploration_constant = exploration_constant, init_Q=special_Q, init_N=special_N, estimate_value = RolloutEstimator(RandomSolver(), max_depth=-1))

    global planner = solve(solver, mdp)

    state = StateCen(gridmap, robots_states, [0,0,0], 0)

    return model, state
end



function agent_step!(model, gridmap, planner, state, visualisation)
    extent = size(gridmap)
    nb_robots = length(state.robots_states)
    robots = [model[i] for i in 1:nb_robots]

    vis_range = robots[1].vis_range
    abmproperties(model).rollout_parameters.timestamp_rollout = state.step

    global a,info = action_info(planner, state)

    if visualisation
        inchrome(D3Tree(info[:tree], init_expand=4))
    end

    next_robots_states = Vector{RobotState}(undef, nb_robots)
    next_gridmap = deepcopy(gridmap)
    step = state.step + 1

    for robot in robots
        next_robots_states[robot.id] = deepcopy(RobotState(robot.id, robot.pos))
    end

    all_robots_pos = [robot.pos for robot in robots]

    for robot in robots
        id = robot.id
        action = a.directions_vector[id].direction
        pos = robot.pos
        
        next_pos,_ = compute_new_pos(next_gridmap, id, all_robots_pos, 1,  action)
        move_agent!(robot, next_pos, model)
        next_robots_states[id] = RobotState(id, next_pos)
        all_robots_pos[id] = next_pos

        obstacles = nearby_obstacles(robot, model, vis_range)
        obstacles_pos = [element.pos for element in obstacles]
        gridmap_update!(next_gridmap, 0, robot.id, all_robots_pos, vis_range, obstacles_pos, model)
    
    end

    return StateCen(next_gridmap, next_robots_states, [0,0,0], step)
end