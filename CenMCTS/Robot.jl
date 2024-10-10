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

    # initialize model
    space = GridSpace(extent, periodic = false, metric = :euclidean)
    rng = Random.MersenneTwister(seed)  # reproducibility
    scheduler = Schedulers.fastest  # they are executed semi-synchronously,
                # in order of their indexing

    D = length(extent)

    T1 = Tuple(i for i in 1:begin_zone[1])
    T2 = Tuple(i for i in 1:begin_zone[2])


    # TODO change to UnremovableABM
    model = AgentBasedModel(Robot{D}, space; agent_step!,
        rng = rng,
        scheduler = scheduler
    )

    for n ∈ 1:N
        pos = (rand(T1),rand(T2))
        id = n
        isObstacle = false
        # alive = true
        agent = Robot{D}(id, pos, vis_range, com_range, isObstacle)
        add_agent!(agent, pos, model)
    end

    return model
end



function compute_new_pos!(gridmap::MMatrix, vis_range::Int, pos::Tuple, other_robots_poses::Vector, action::action_robot; transition = false, proba_obs = 0.0, rng = nothing, distribution = nothing)

    extent = size(gridmap)
    pos_action = (vis_range .* action.action) .+ pos
    ray = raytracing(pos, pos_action, vis_range)
    x,y = ray[1][1],ray[1][2]

    if length(ray)==1
        return (x,y), (0,0)
    
    else
        for element in ray[2:end]
            x_prev, y_prev = x,y 
            x,y = element[1], element[2]

            # hors zone 
            if (x > extent[1]) || (y > extent[2]) || (x < 1) || (y < 1)
                return (x_prev, y_prev), (0,0)
            end

            # cellule inconnue, uniquement cas transition
            if transition
                if gridmap[x,y] == -2
                    gridmap[x,y] = rand(rng, distribution)
                end
            end

            
            if gridmap[x,y] == -1 
                return (x_prev, y_prev), (x,y)
            elseif (x,y) ∈ other_robots_poses
                return (x_prev, y_prev), (0,0)
            end
        end
    end
    return (x,y), (0,0)
end


function agent_step!(model, gridmap, planner, state, visualisation)
    extent = size(gridmap)
    nb_robots = length(state.robot_list)
    robots = [model[i] for i in 1:nb_robots]

    vis_range = robots[1].vis_range

    global a,info = action_info(planner, state)

    if visualisation
        inchrome(D3Tree(info[:tree], init_expand=4))
    end

    next_robots_states = Vector{robot_state}(undef, nb_robots)
    next_gridmap = MMatrix{extent[1],extent[2]}(gridmap)
    nb_coups = state.nb_coups + 1

    for robot in robots
        next_robots_states[robot.id] = robot_state(robot.id, robot.pos)
    end

    for robot in robots
        id = robot.id
        action = filter(item->item.id == id, a)[1]
        pos = robot.pos
        other_robots_poses = [rob.pos for rob in next_robots_states if rob.id != id]

        next_pos,_ = compute_new_pos!(gridmap, vis_range, pos, other_robots_poses, action)
        move_agent!(robot,next_pos,model)
        next_robots_states[id] = robot_state(id, next_pos)
        robot.pos = next_pos

        obstacles = filter(obj -> obj.isObstacle, collect(nearby_agents(robot, model, robot.vis_range)))
        obstacles_poses = [element.pos for element in obstacles]
        gridmap_update!(next_gridmap, robot.id, next_pos, other_robots_poses, vis_range, obstacles_poses, model)
    end
    
    return mdp_state(next_robots_states, next_gridmap, state.seen_gridmap, nb_coups)
end