include("../src/Packages.jl")
include("../src/Types.jl")
include("Figure.jl")
include("Robot.jl")
include("MDP.jl")
include("../src/Maps.jl")
include("Robot.jl")
include("../src/Sensor.jl")
include("../src/FrontierDetection.jl")
include("Optimization.jl")
include("../src/Communication.jl")
include("../src/Utils.jl")

global wait_for_key(prompt) = (print(stdout, prompt); read(stdin, 1); nothing)

function run(; 
    alpha_state = 0.5, 
    k_state = 1.0, 
    alpha_action = 1.0,
    k_action = 1.0,
    exploration_constant = 0.5, 
    n_iterations = 20, 
    keep_tree = false, 
    discount = 0.85, 
    nb_obstacles = [0], 
    nb_robots = 3,
    extent = (40,40),
    vis_figure = false,
    vis_tree = false,
    depth = 100,
    max_time = 60.0, 
    show_progress = false,
    max_steps = 500,
    num_map = -1,
    nb_blocs = 0,
    com_range = 10,
    vis_range = 3,
    fct_proba = compute_q,
    fct_sequence = state_best_average_action,
    nb_sequence = 3,
    proba_communication = 1.0,
    rollout = "frontiers",
    alpha = 0.01,
    file = "",
    begin_zone = (5,5),
    fct_reward = simple_reward,
    filtering_info = false,
    fct_communication = simple_communication!,
    id_expe = 0
    )

    if typeof(fct_communication) == String
        fct_communication = getfield(Main, Symbol(fct_communication))
    end
    if typeof(fct_proba) == String
        fct_proba = getfield(Main, Symbol(fct_proba))
    end
    if typeof(fct_sequence) == String
        fct_sequence = getfield(Main, Symbol(fct_sequence))
    end
    if typeof(fct_reward) == String
        fct_reward = getfield(Main, Symbol(fct_reward))
    end

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

    global model = initialize_model(
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
        begin_zone = begin_zone,
        vis_range = vis_range,    
        com_range = com_range,
        invisible_cells = invisible_cells,
        nb_blocs = nb_blocs, 
        fct_reward = fct_reward,
        filtering_info = filtering_info,
        rollout = rollout
    )

    robots = [model[i] for i in 1:nb_robots]

    
    if vis_figure
        observ_map = Array{Observable}(undef, nb_robots)
        observ_pos_list = Vector{Vector{Observable}}(undef,nb_robots)
        observ_traj_list = [Observable([Point2f(0.5,0.5)]) for i in 1:nb_robots]
        for (i,r) in enumerate(robots)
            id = r.id
            observ_pos_list[id] = Vector{Observable}(undef, nb_robots)
            observ_map[id] = Observable(Matrix(robots[1].state.gridmap))
            push!(observ_traj_list[id][], Point2f(r.pos))
            for i in 1:nb_robots
                observ_pos_list[id][i] = Observable(r.plans[i].state.pos)
            end
        end
        global f = initialise_figure(observ_map, observ_pos_list, observ_traj_list)
    
    end
     
    nb_steps = 0
    max_knowledge = maximum([r.state.known_cells for r in robots])

    walkmap = BitArray{2}(trues(extent))
    obs = [r.pos for r in collect(nearby_obstacles((1,1),model,100))]
    for p in obs
        walkmap[p[1],p[2]] = false
    end
    pathfinder = Agents.Pathfinding.AStar(abmspace(model), walkmap=walkmap)

    distribution_communication = SparseCat([true, false], [proba_communication, 1-proba_communication])

    while (max_knowledge < (extent[1]*extent[2]) - abmproperties(model).invisible_cells[1]) && (nb_steps < max_steps)

        nb_steps += 1
        
        @threads for robot in robots 
            agents_simulate!(robot, model, alpha, 1-(nb_steps-1)/(extent[1]*extent[2]); fct_proba = fct_proba, fct_sequence = fct_sequence, nb_sequence = nb_sequence, distribution_communication = distribution_communication, fct_communication = fct_communication)
        end

        for robot in robots
            agent_step!(robot, model, vis_tree)

            if vis_figure
                for i in 1:nb_robots
                    observ_pos_list[robot.id][i][] = robot.plans[i].state.pos  
                end              
                observ_traj_list[robot.id][] = push!(observ_traj_list[robot.id][], Point2f(robot.pos))
                observ_map[robot.id][] = robot.state.gridmap
            end

            
        end
        max_knowledge = maximum([r.state.known_cells for r in robots])

        if id_expe!=0 && file != ""
            add_metrics(model, pathfinder, file, id_expe;
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
            com_range = com_range,
            fct_proba = fct_proba,
            fct_sequence = fct_sequence,
            nb_sequence = nb_sequence,
            proba_communication = proba_communication,
            alpha = alpha
            )
        end

    end        
    return nb_steps
end

function describe_robot(robot::Robot)
    println("id = $(robot.id)")
    println("pos = $(robot.pos)")
    println("range of communication = $(robot.com_range), range of vision = $(robot.vis_range)")
    println("in a rollout = $(robot.in_rollout)")
    println("State : ")
    println("nombre de coups = $(robot.state.step)")
    println("all states from this robot point of view =")
    for p in robot.state.space_state.robots_plans
        println("id = $(p.state.id), pos = $(p.state.pos)")
    end
    println("gridmap : ")
    _print_gridmap(robot.state.space_state.gridmap, [p.state for p in robot.state.space_state.robots_plans])
end



function list_pos(gridmap::MMatrix, id::Int, robots_pos::Union{Vector, SizedVector}, vis_range::Int, action_sequence::MutableLinkedList{ActionDec})
    L = Vector{Tuple{Int,Int}}(undef, length(action_sequence)+1)
    L[1] = robots_pos[id]
    for (i,a) in enumerate(action_sequence)
        pos,_ = compute_new_pos(gridmap, id, robots_pos, vis_range, a.direction)
        robots_pos[id] = pos
        L[i+1] = pos
    end
    return L
end


function add_metrics(model::StandardABM, pathfinder::Pathfinding.AStar{2}, file::String, id_expe::Int;
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
    max_steps = 300,
    num_map = 2,
    com_range = 10,
    fct_proba = compute_q,
    fct_sequence = state_best_average_action,
    nb_sequence = 3,
    proba_communication = 1.0,
    alpha = 0.01
    )
    nb_robots = length(model[1].plans)
    robots = [model[i] for i in 1:nb_robots]
    extent = size(robots[1].state.gridmap)

    percent_of_map = zeros(length(robots)+1)
    astar_distances = zeros((length(robots), length(robots)))
    euclidean_distances = zeros((length(robots), length(robots)))
    
    tmp = 0
    for i in 1:extent[1]
        for j in 1:extent[2]
            v = [abmproperties(model).seen_all_gridmap[r][i,j] for r in 1:nb_robots]
            if any(v.>0)
                tmp+=1
            end
        end
    end

    percent_of_map[end] = tmp/(extent[1]*extent[2]-abmproperties(model).nb_obstacles[1])
    df = DataFrame("nb_steps" => robots[1].state.step, "percent_of_map_all" => percent_of_map[end], "positions" => [[r.pos for r in robots]])

    for robot in robots

        percent_of_map[robot.id] = count(x-> x>0, [abmproperties(model).seen_all_gridmap[robot.id][i,j] for i in 1:extent[1] for j in 1:extent[2]])/(extent[1]*extent[2]-abmproperties(model).nb_obstacles[1])

        for robot_prime in filter(x->x.id!=robot.id,robots[robot.id+1:end])
            route = plan_route!(robot, robot_prime.pos, pathfinder)
            if !isempty(route)
                astar_distances[robot.id, robot_prime.id] = length(route)
            else
                astar_distances[robot.id, robot_prime.id] = Inf
            end
            astar_distances[robot_prime.id, robot.id] = astar_distances[robot.id, robot_prime.id]
            euclidean_distances[robot.id, robot_prime.id] = sqrt((robot.pos[1]-robot_prime.pos[1])^2 + (robot.pos[2]-robot_prime.pos[2])^2)
            euclidean_distances[robot_prime.id, robot.id] = euclidean_distances[robot.id, robot_prime.id]
        end

        rewards = Tuple{ActionDec, Float64}[]
        a_indexes = robot.planner.tree.children[1]
        for a_index in a_indexes
            push!(rewards,(robot.planner.tree.a_labels[a_index], robot.planner.tree.q[a_index]))
        end

        df = innerjoin(df, DataFrame("nb_steps" => robots[1].state.step, "percent_of_map_$(robot.id)" => percent_of_map[robot.id], "astar_distances_$(robot.id)" => [astar_distances[robot.id,:]], "euclidean_distances_$(robot.id)" =>[euclidean_distances[robot.id,:]], "seen_gridmap_$(robot.id)" => [abmproperties(model).seen_all_gridmap[robot.id][:,:]], "gridmap_$(robot.id)" => [robot.state.gridmap], "rewards_$(robot.id)" => [rewards], "action_$(robot.id)" => robot.last_action), on = "nb_steps")
        
    end

    write_header = false
    if  robots[1].state.step == 1
        write_header = true
    end
    CSV.write(file*"$(id_expe).csv", df, delim = ';', header = write_header, append=true)

end


function save_states(nb_robots::Int, file::String)
    df = DataFrame("ids" => [i for i in 1:nb_robots])
    df[!, "pos"] =  [model[i].pos for i in 1:nb_robots]
    df[!, "gridmap"] =  [model[i].state.gridmap for i in 1:nb_robots]
    df[!, "robots_states"] = [model[i].state.robots_states for i in 1:nb_robots]
    df[!, "known_cells"] =  [model[i].state.known_cells for i in 1:nb_robots]
    df[!, "seen_cells"] =  [model[i].state.seen_cells for i in 1:nb_robots]
    for j in 1:nb_robots
        df[!, "sequences_of_$j"] = [model[i].plans[j].best_sequences for i in 1:nb_robots]
        df[!, "assigned_proba_of_$j"] = [model[i].plans[j].assigned_proba for i in 1:nb_robots]
        df[!, "timestamp_of_$j"] = [model[i].plans[j].timestamp for i in 1:nb_robots]
    end
    CSV.write("$(file).csv", df, delim=";", header = true, append=true)
end