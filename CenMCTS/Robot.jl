MCTS.node_tag(s::GWPos) = "[$(s[1]),$(s[2])]"
MCTS.node_tag(a::Symbol) = "[$a]"


function initialize_model(;
    N = 3,               # number of agents
    extent = (20,20),    # size of the world
    begin_zone = (3,1),   # beinning zone for robots
    vis_range = 2.0,       # visibility range
    com_range = 2.0,       # communication range
    nb_obstacles = 0,
    invisible_cells = 0,
    seed = 1               # random seed
)

    nb_robots = N
    # initialize model
    space = GridSpace(extent, periodic = false, metric = :euclidean)
    rng = Random.MersenneTwister(seed)  # reproducibility
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


    # TODO change to UnremovableABM
    model = AgentBasedModel(Union{Robot, Obstacle}, space; agent_step!,
        rng = rng,
        scheduler = scheduler,
        properties = properties
    )

    for n ∈ 1:N
        pos = (rand(T1),rand(T2))
        id = n
        agent = RobotCen{D}(id, pos, vis_range)
        add_agent!(agent, pos, model)
    end

    return model
end



function agent_step!(model, gridmap, planner, state, visualisation)
    extent = size(gridmap)
    nb_robots = length(state.robots_states)
    robots = [model[i] for i in 1:nb_robots]

    vis_range = robots[1].vis_range

    global a,info = action_info(planner, state)

    if visualisation
        inchrome(D3Tree(info[:tree], init_expand=4))
    end

    next_robots_states = Vector{RobotState}(undef, nb_robots)
    next_gridmap = MMatrix{extent[1],extent[2]}(gridmap)
    nb_coups = state.nb_coups + 1

    for robot in robots
        next_robots_states[robot.id] = RobotState(robot.id, robot.pos)
    end

    all_robots_pos = [robot.pos for robot in robots]

    for robot in robots
        id = robot.id
        action = a.directions[id].direction
        pos = robot.pos
        
        next_pos,_ = compute_new_pos(gridmap, id, all_robots_pos, vis_range,  action)
        move_agent!(robot,next_pos,model)
        next_robots_states[id] = RobotState(id, next_pos)
        robot.pos = next_pos
        all_robots_pos[id] = next_pos

        obstacles = nearby_obstacles(robot, model, robot.vis_range)
        obstacles_pos = [element.pos for element in obstacles]
        gridmap_update!(next_gridmap, 0, robot.id, all_robots_pos, vis_range, obstacles_pos, model)
    end
    
    return StateCen(next_gridmap, next_robots_states, nb_coups)
end