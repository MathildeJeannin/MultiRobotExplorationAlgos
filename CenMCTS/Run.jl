include("../src/Packages.jl")
include("../src/Types.jl")
include("Figure.jl")
include("Robot.jl")
include("MDP.jl")
include("../src/Maps.jl")
include("Robot.jl")
include("../src/Sensor.jl")
include("../src/FrontierDetection.jl")
include("../src/Utils.jl")


function run(; 
    alpha_state = 1.0, 
    k_state = 1.0, 
    alpha_action = 3.0,
    k_action = 1.0,
    exploration_constant = 20.0, 
    n_iterations = 2500, 
    keep_tree = false, 
    discount = 0.85, 
    nb_obstacles = [0], 
    nb_robots = 3,
    extent = (20,20),
    vis_figure = false,
    vis_tree = false,
    depth = 100,
    max_time = 60.0, 
    show_progress = false,
    max_steps = 500,
    num_map = -1, 
    com_range = 10,
    penalite = false,
    id_expe = 0,
    file = "",
    nb_blocs = 0,
    begin_zone = (5,5),
    reward_function = all_move_reward
    )

    vis_range = 3
    log = []
    invisible_cells = [0]
    if num_map > 0 
        f = open("./src/maps/map$num_map.txt", "r")
        line_extent = readline(f)
        line_invisible_cells = readline(f)
        close(f)
        str_extent = split(line_extent, ";")
        extent = (parse(Int64, str_extent[1]),parse(Int64, str_extent[2]))
        invisible_cells = [parse(Int64, line_invisible_cells)]
        nb_obstacles = [countlines("./src/maps/map$(num_map).txt") - 2]
    end
    
    global model, state = initialize_model(;
        nb_robots = nb_robots,                 # number of agents
        extent = extent,               # size of the world
        vis_range = vis_range,       # visibility range
        com_range = com_range,       # communication range
        seed = rand(1:10^39), 
        nb_obstacles = nb_obstacles, 
        invisible_cells = invisible_cells,
        num_map = num_map,
        discount = discount,
        n_iterations = n_iterations,
        depth = depth,
        alpha_state = alpha_state, 
        k_state = k_state, 
        alpha_action = alpha_action,
        k_action = k_action,
        exploration_constant = exploration_constant,  
        keep_tree = keep_tree, 
        max_time = max_time, 
        show_progress = show_progress,
        max_steps = max_steps,
        nb_blocs = nb_blocs,
        begin_zone = begin_zone,
        reward_function = all_move_reward
    )

    robots = [model[i] for i in 1:nb_robots]
    gridmap = state.gridmap

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
    while (count(i->i!=-2, gridmap) != (extent[1]*extent[2]-invisible_cells[1])) && (nb_steps < max_steps)
        nb_steps += 1
        state = agent_step!(model, gridmap, planner, state, vis_tree)
        gridmap = state.gridmap
        for (j,rob) in enumerate(robots)
            id = rob.id
            if vis_figure
                observ_pos_list[id][] = rob.pos
                observ_traj_list[id][] = push!(observ_traj_list[id][], Point2f(rob.pos))
            end
        end
        
        vis_figure ? observ_map[] = Matrix(gridmap) : nothing
        if id_expe > 0 && file != ""
            add_metrics(model, state, pathfinder, file, id_expe;
                alpha_state = alpha_state, 
                k_state = k_state , 
                alpha_action = alpha_action ,
                k_action = k_action,
                exploration_constant = exploration_constant, 
                n_iterations = n_iterations, 
                keep_tree = keep_tree, 
                discount = discount, 
                nb_obstacles = nb_obstacles, 
                nb_robots = nb_robots,
                extent = extent,
                depth = depth,
                max_time = max_time, 
                max_steps = max_steps,
                num_map = num_map,
                invisible_cells = invisible_cells[1]
            )
        end
    end

    return nb_steps

end


function add_metrics(model::StandardABM, state::StateCen, pathfinder::Pathfinding.AStar{2}, file::String, id_expe::Int;
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
    extent = size(state.gridmap)

    percent_of_map = zeros(1)
    astar_distances = zeros((length(robots), length(robots)))
    euclidean_distances = zeros((length(robots), length(robots)))
    
    percent_of_map[1] = count(x->x!=-2, state.gridmap)/(extent[1]*extent[2]-invisible_cells)
    df = DataFrame("nbsteps" => state.step, "percentofmapall" => percent_of_map[1], "seen_gridmap" => [abmproperties(model).seen_all_gridmap])

    for robot in robots

        for robot_prime in filter(x->x.id!=robot.id,robots[robot.id+1:end])
            astar_distances[robot.id, robot_prime.id] = length(plan_route!(robot, robot_prime.pos, pathfinder))
            astar_distances[robot_prime.id, robot.id] = astar_distances[robot.id, robot_prime.id]
            euclidean_distances[robot.id, robot_prime.id] = sqrt((robot.pos[1]-robot_prime.pos[1])^2 + (robot.pos[2]-robot_prime.pos[2])^2)
            euclidean_distances[robot_prime.id, robot.id] = euclidean_distances[robot.id, robot_prime.id]
        end

        df = innerjoin(df, DataFrame("nbsteps" => state.step, "astardistances$(robot.id)" => [astar_distances[robot.id,:]], "euclideandistances$(robot.id)" =>[euclidean_distances[robot.id,:]]), on = "nbsteps")
        
    end

    write_header = false
    if  state.step == 1
        write_header = true
    end
    CSV.write(file*"$(id_expe).csv", df, delim = ';', header = write_header, append=true)

end