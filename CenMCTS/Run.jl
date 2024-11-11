include("../src/Packages.jl")
include("../src/Types.jl")
include("Figure.jl")
include("Robot.jl")
include("MDP.jl")
include("../src/Maps.jl")
include("Robot.jl")
include("../src/Sensor.jl")

function run(; 
    alpha_state = 1.0, 
    k_state = 500.0, 
    alpha_action = 1.0,
    k_action = 1.0,
    exploration_constant = 1.0, 
    n_iterations = 1000, 
    keep_tree = false, 
    discount = 0.85, 
    nb_obstacles = 0, 
    nb_robots = 3,
    extent = (15,15),
    vis_figure = false,
    vis_tree = false,
    depth = 50,
    max_time = 60.0, 
    show_progress = false,
    max_steps = 100,
    num_map = -1, 
    com_range = 10,
    penalite = false,
    id_expe = 0
    )

    global use_penalite = penalite

    vis_range = 3
    log = []
    invisible_cells = 0
    if num_map > 0 
        f = open("../src/maps/map$num_map.txt", "r")
        line_extent = readline(f)
        line_triche = readline(f)
        close(f)
        str_extent = split(line_extent, ";")
        extent = (parse(Int64, str_extent[1]),parse(Int64, str_extent[2]))
        invisible_cells = parse(Int64, line_triche)
        nb_obstacles = countlines("../src/maps/map$(num_map).txt") - 2
    end

    global gridmap = MMatrix{extent[1],extent[2]}(Int8.(-2*ones(Int8, extent)))
    seen_gridmap = MMatrix{extent[1],extent[2]}(Int8.(zeros(Int8, extent)))
    
    global model = initialize_model(;
        N = nb_robots,                 # number of agents
        extent = extent,               # size of the world
        begin_zone = (1,1),
        vis_range = vis_range,       # visibility range
        com_range = com_range,       # communication range
        seed = rand(1:10^39), 
        nb_obstacles = nb_obstacles, 
        invisible_cells = invisible_cells
    )

    if num_map > 0
        add_map(model, num_map, nb_robots)
    elseif num_map < 0
        invisible_cells = add_simple_obstacles(model, extent; N = 1)
    else
        add_obstacles(model; N = nb_obstacles, extent = extent)
    end

    allObjects = allagents(model)

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
    end

    possible_actions = compute_actions(nb_robots)

    mdp = RobotMDP(vis_range, nb_obstacles, discount, possible_actions)

    solver = DPWSolver(n_iterations = n_iterations, depth = depth, max_time = max_time, keep_tree = keep_tree, show_progress = show_progress, enable_action_pw = true, enable_state_pw = true, tree_in_info = true, alpha_state = alpha_state, k_state = k_state, alpha_action = alpha_action, k_action = alpha_action, exploration_constant = exploration_constant)

    global planner = solve(solver, mdp)

    state = StateCen(gridmap, robots_states, 0)

    push!(log, [copy(state.robots_states), copy(state.gridmap)])

    if vis_figure
        observ_map = Observable(Matrix(gridmap))
        observ_pos_list = Array{Observable}(undef,nb_robots)
        observ_traj_list = [Observable([Point2f(0.5,0.5)]) for i in 1:nb_robots]
        for (i,r) in enumerate(robots)
            id = r.id
            observ_pos_list[id] = Observable(r.pos)
            push!(observ_traj_list[id][], Point2f(r.pos))
        end
        initialise_figure([observ_map], observ_pos_list, observ_traj_list)
    end

    walkmap = BitArray{2}(trues(extent))
    obs = [r.pos for r in collect(nearby_obstacles((1,1),model,100))]
    for p in obs
        walkmap[p[1],p[2]] = false
    end
    pathfinder = Agents.Pathfinding.AStar(abmspace(model), walkmap=walkmap)

    nb_steps = 0
    while (count(i->i!=-2, gridmap) != (extent[1]*extent[2]-invisible_cells)) && (nb_steps < max_steps)
    # while (count(i->i!=-2, gridmap) != (extent[1]*extent[2])) && nb_steps < max_steps
        nb_steps += 1
        t0 = time()
        state = agent_step!(model, gridmap, planner, state, vis_tree)
        gridmap = state.gridmap
        for (j,rob) in enumerate(robots)
            id = rob.id
            if vis_figure
                observ_pos_list[id][] = rob.pos
                observ_traj_list[id][] = push!(observ_traj_list[id][], Point2f(rob.pos))
            end
        end

        # if nb_steps > max_steps/3
        #     check_for_invisible_obstacles!(gridmap)
        # end
        
        vis_figure ? observ_map[] = Matrix(gridmap) : nothing
        # add_metrics(model, state, pathfinder, id_expe;
        #     alpha_state = alpha_state, 
        #     k_state = k_state , 
        #     alpha_action = alpha_action ,
        #     k_action = k_action,
        #     exploration_constant = exploration_constant, 
        #     n_iterations = n_iterations, 
        #     keep_tree = keep_tree, 
        #     discount = discount, 
        #     nb_obstacles = nb_obstacles, 
        #     nb_robots = nb_robots,
        #     extent = extent,
        #     depth = depth,
        #     max_time = max_time, 
        #     max_steps = max_steps,
        #     num_map = num_map,
        #     invisible_cells = invisible_cells
        # )
    end

    return nb_steps, abmproperties(model).seen_all_gridmap

end



function _compare_log_infotree(log, tree)
    log_tree = [tree.s_labels[N] for N in findall(x->x>1000, tree.total_n)]
    nb_robots = length(log_tree[1].robot_list)

    for i in 1:min(length(log), length(log_tree))
        println("Step $(i)\n")
        println(log_tree[i].gridmap == log[i][2])
        println("\ntree : ")
        _print_gridmap(log_tree[i].gridmap, [log_tree[i].robot_list[j].pos for j in 1:nb_robots])
        println("log : ")
        _print_gridmap(log[i][2], [log[i][1][j].pos for j in 1:nb_robots])
    end
end


function _print_log(log)
    println("Log : ")
    for element in log
        gridmap = element[2]
        poses = [element[1][i].pos for i in 1:2]
        _print_gridmap(gridmap, poses)
    end
end


function _print_tree(tree, param_findall)
    N = param_findall
    println("Tree : ")
    for N in findall(x->x>1000, tree.total_n)
        gridmap = tree.s_labels[N].gridmap
        poses = [tree.s_labels[N].robots_states[i].pos for i in 1:length(tree.s_labels[N].robots_states)]
        _print_gridmap(gridmap, poses)
    end
end


function add_metrics(model::StandardABM, state::StateCen, pathfinder::Pathfinding.AStar{2}, id_expe::Int;
    alpha_state = 1.0, 
    k_state = 500.0, 
    alpha_action = 1.0,
    k_action = 1.0,
    exploration_constant = 1.0, 
    n_iterations = 500, 
    keep_tree = false, 
    discount = 0.85, 
    nb_obstacles = 0, 
    nb_robots = 3,
    extent = (15,15),
    depth = 50,
    max_time = 60.0, 
    max_steps = 100,
    num_map = 2,
    invisible_cells = invisible_cells
    )
    robots = [model[i] for i in 1:nb_robots]
    extent = size(gridmap)

    percent_of_map = zeros(1)
    astar_distances = zeros((length(robots), length(robots)))
    euclidean_distances = zeros((length(robots), length(robots)))
    
    percent_of_map[1] = count(x->x!=-2, gridmap)/(extent[1]*extent[2]-invisible_cells)
    df = DataFrame("nbsteps" => state.nb_coups, "percentofmapall" => percent_of_map[1])

    for robot in robots

        for robot_prime in filter(x->x.id!=robot.id,robots[robot.id+1:end])
            astar_distances[robot.id, robot_prime.id] = length(plan_route!(robot, robot_prime.pos, pathfinder))
            astar_distances[robot_prime.id, robot.id] = astar_distances[robot.id, robot_prime.id]
            euclidean_distances[robot.id, robot_prime.id] = sqrt((robot.pos[1]-robot_prime.pos[1])^2 + (robot.pos[2]-robot_prime.pos[2])^2)
            euclidean_distances[robot_prime.id, robot.id] = euclidean_distances[robot.id, robot_prime.id]
        end

        df = innerjoin(df, DataFrame("nbsteps" => state.nb_coups, "astardistances$(robot.id)" => [astar_distances[robot.id,:]], "euclideandistances$(robot.id)" =>[euclidean_distances[robot.id,:]], "gridmap$(robot.id)" => [gridmap]), on = "nbsteps")
        
    end

    write_header = false
    if  state.nb_coups == 1
        write_header = true
    end
    # CSV.write("Logs/$alpha$(alpha_action)_k$(k_action)_map$(num_map)/$(N).csv", df, delim = ';', header = write_header, append=true)
    # println(df)

end