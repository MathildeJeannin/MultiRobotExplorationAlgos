MCTS.node_tag(s::GWPos) = "[$(s[1]),$(s[2])]"
MCTS.node_tag(a::Symbol) = "[$a]"


function initialize_model(;
    N = 3,               # number of agents
    extent = (20,20),    # size of the world
    begin_zone = (3,1),   # beinning zone for robots
    vis_range = 2.0,       # visibility range
    com_range = 2.0,       # communication range
    nb_obstacles = 0,
    invisible_cells = [0],
    seed = 1,               # random seed
    num_map = 0,
    discount = 0.85,
    n_iterations = 1500,
    depth = 50,
    alpha_state = 1.0, 
    k_state = 500.0, 
    alpha_action = 1.0,
    k_action = 1.0,
    exploration_constant = 1.0,
    keep_tree = false,
    max_time = 60.0,
    show_progress = false,
    max_steps = 100,
    nb_blocs = 0
)

    nb_robots = N
    # initialize model
    space = GridSpace(extent, periodic = false, metric = :euclidean)
    rng = Random.MersenneTwister(seed)  # reproducibility
    scheduler = Schedulers.fastest  # they are executed semi-synchronously,
                # in order of their indexing

    D = length(extent)

    global gridmap = MMatrix{extent[1],extent[2]}(Int64.(-2*ones(Int64, extent)))
    seen_gridmap = MMatrix{extent[1],extent[2]}(Int64.(zeros(Int64, extent)))

    T1 = Tuple(i for i in 1:begin_zone[1])
    T2 = Tuple(i for i in 1:begin_zone[2])

    frontiers = Set()
    actions_sequence = Vector{ActionCen}(undef, 0)
    rollout_parameters = RolloutInfo(false, 1, frontiers, [(0,0) for i in 1:nb_robots], MVector{0,Nothing}())

    properties = (
        seen_all_gridmap = MVector{nb_robots, MMatrix}(MMatrix{extent[1],extent[2]}(Int64.(zeros(Int64, extent))) for i in 1:nb_robots),
        nb_obstacles, 
        invisible_cells,
        nb_robots,
        rollout_parameters
    )


    # TODO change to UnremovableABM
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

    walkmap = BitArray{2}(trues(extent[1],extent[2]))
    pathfinder = Agents.Pathfinding.AStar(abmspace(model), walkmap=walkmap)

    pos = Vector{Tuple{Int,Int}}(undef, nb_robots)
    for n ∈ 1:N
        pos[n] = (rand(T1),rand(T2))
        while !isempty(ids_in_position(pos[n], model))
            pos[n] = (rand(T1),rand(T2))
        end
    end

    for n in 1:N
        id = n
        agent = RobotCen{D}(id, pos[n], vis_range, pathfinder, Set())
        add_agent!(agent, pos[n], model)
    end

    robots_states = Vector{RobotState}(undef, nb_robots)
    robots = [model[i] for i in 1:nb_robots]

    for (i,r) in enumerate(robots)
        id = r.id
        robots_states[id] = RobotState(r.id, r.pos)
    end

    all_robots_pos = [r.pos for r in robots]
    for robot in robots
        obstacles = nearby_obstacles(robot, model, robot.vis_range)
        obstacles_pos = [element.pos for element in obstacles]

        gridmap_update!(gridmap, 0, robot.id, all_robots_pos, robot.vis_range, obstacles_pos, model)

        pathfinder_update!(robot.pathfinder, gridmap)

        frontiers = frontierDetection(robot.id, robot.pos, robot.vis_range, gridmap, all_robots_pos, abmproperties(model).rollout_parameters.frontiers; need_repartition=false)
        abmproperties(model).rollout_parameters.frontiers = union(abmproperties(model).rollout_parameters.frontiers, frontiers)
    end

    possible_actions = compute_actions_cenMCTS(nb_robots)

    mdp = RobotMDP(vis_range, nb_obstacles[1], discount, possible_actions)

    frontier_policy = FrontierPolicy(mdp)

    solver = DPWSolver(n_iterations = n_iterations, depth = depth, max_time = max_time, keep_tree = keep_tree, show_progress = show_progress, enable_action_pw = true, enable_state_pw = false, tree_in_info = true, alpha_state = alpha_state, k_state = k_state, alpha_action = alpha_action, k_action = k_action, exploration_constant = exploration_constant, init_Q=special_Q, estimate_value = RolloutEstimator(RandomSolver(), max_depth=-1))

    global planner = solve(solver, mdp)

    state = StateCen(gridmap, robots_states, [0,0,0], 0)

    return model, state
end



function agent_step!(model, gridmap, planner, state, visualisation)
    extent = size(gridmap)
    nb_robots = length(state.robots_states)
    robots = [model[i] for i in 1:nb_robots]

    vis_range = robots[1].vis_range
    abmproperties(model).rollout_parameters.debut_rollout = state.nb_coups

    global a,info = action_info(planner, state)

    if visualisation
        inchrome(D3Tree(info[:tree], init_expand=4))
    end

    next_robots_states = Vector{RobotState}(undef, nb_robots)
    next_gridmap = deepcopy(gridmap)
    nb_coups = state.nb_coups + 1

    for robot in robots
        next_robots_states[robot.id] = deepcopy(RobotState(robot.id, robot.pos))
    end

    all_robots_pos = [robot.pos for robot in robots]

    for robot in robots
        id = robot.id
        action = a.directions[id].direction
        pos = robot.pos
        
        next_pos,_ = compute_new_pos(next_gridmap, id, all_robots_pos, 1,  action)
        move_agent!(robot, next_pos, model)
        next_robots_states[id] = RobotState(id, next_pos)
        all_robots_pos[id] = next_pos

        obstacles = nearby_obstacles(robot, model, robot.vis_range)
        obstacles_pos = [element.pos for element in obstacles]
        gridmap_update!(next_gridmap, 0, robot.id, all_robots_pos, vis_range, obstacles_pos, model)

        
        frontiers = frontierDetection(robot.id, robot.pos, robot.vis_range, next_gridmap, all_robots_pos, abmproperties(model).rollout_parameters.frontiers; need_repartition=false)
        abmproperties(model).rollout_parameters.frontiers = union(abmproperties(model).rollout_parameters.frontiers, frontiers)
    
    end

    for robot in robots
        pathfinder_update!(robot.pathfinder, next_gridmap)
    end

    return StateCen(next_gridmap, next_robots_states, [0,0,0], nb_coups)
end