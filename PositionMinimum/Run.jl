include("../src/Packages.jl")
include("../src/Types.jl")
include("Figure.jl")
include("Robot.jl")
include("../src/Maps.jl")
include("Robot.jl")
include("../src/Sensor.jl")
include("../src/Communication.jl")
include("../src/FrontierDetection.jl")
include("PositionMinimum.jl")

wait_for_key(prompt) = (print(stdout, prompt); read(stdin, 1); nothing)

function run(;
    nb_obstacles = 0, 
    nb_robots = 3,
    extent = (15,15),
    vis_figure = false,
    max_steps = 100,
    num_map = 2,
    com_range = 10,
    id_expe = 0
    )

    vis_range = 3
    invisible_cells = [0]
    if num_map > 0 
        f = open("/home/mathilde/Documents/These/Codes/SimulateursExploration/src/maps/map$num_map.txt", "r")
        line_extent = readline(f)
        line_triche = readline(f)
        close(f)
        str_extent = split(line_extent, ";")
        extent = (parse(Int64, str_extent[1]),parse(Int64, str_extent[2]))
        invisible_cells = [parse(Int64, line_triche)]
        nb_obstacles = countlines("/home/mathilde/Documents/These/Codes/SimulateursExploration/src/maps/map$(num_map).txt") - 2
    end

    global model = initialize_model(
        nb_robots,
        extent,
        nb_obstacles,
        num_map;
        begin_zone = (1,1),
        vis_range = vis_range,    
        com_range = com_range,
        invisible_cells = invisible_cells
    )

    robots = [model[i] for i in 1:nb_robots]

    if vis_figure
        observ_map = Array{Observable}(undef, nb_robots)
        observ_pos_list = Vector{Vector{Observable}}(undef,nb_robots)
        observ_traj_list = [Observable([Point2f(0.5,0.5)]) for i in 1:nb_robots]
        for (i,r) in enumerate(robots)
            id = r.id
            observ_pos_list[id] = Vector{Observable}(undef, nb_robots)
            observ_map[id] = Observable(Matrix(robots[1].gridmap))
            push!(observ_traj_list[id][], Point2f(r.pos))
            for i in 1:nb_robots
                observ_pos_list[id][i] = Observable(robots[i].pos)
            end
        end
        global f = initialise_figure(observ_map, observ_pos_list, observ_traj_list)
    
    end
    
    nb_steps = 0
    max_knowledge = 0

    walkmap = BitArray{2}(trues(extent))
    obs = [r.pos for r in collect(nearby_obstacles((1,1),model,100))]
    for p in obs
        walkmap[p[1],p[2]] = false
    end
    pathfinder = Agents.Pathfinding.AStar(abmspace(model), walkmap=walkmap)
    # plan_route!(agent, pos, pathfinder)

    while (max_knowledge != (extent[1]*extent[2]-abmproperties(model).invisible_cells[1])) && (nb_steps < max_steps)
        nb_steps += 1

        for robot in robots

            agent_step!(robot, model)

            if vis_figure
                for i in 1:nb_robots
                    observ_pos_list[robot.id][i][] = robot.all_robots_pos[i]  
                end              
                observ_traj_list[robot.id][] = push!(observ_traj_list[robot.id][], Point2f(robot.pos))
                observ_map[robot.id][] = robot.gridmap
            end

            sleep(0.5)

            
        end
        max_knowledge = maximum([count(x -> x != -2, r.gridmap) for r in robots])

        if id_expe!=0
            add_metrics(model, pathfinder, id_expe, nb_steps;
            nb_obstacles = nb_obstacles, 
            nb_robots = nb_robots,
            extent = extent,
            max_time = max_time, 
            max_steps = max_steps,
            num_map = num_map,
            com_range = com_range
            )
        end

    end        
    return nb_steps, abmproperties(model).seen_all_gridmap
end


function add_metrics(model::StandardABM, pathfinder::Pathfinding.AStar{2}, id_expe::Int, nb_step::Int;
    nb_obstacles = 0, 
    nb_robots = 3,
    extent = (15,15),
    depth = 50,
    max_time = 60.0, 
    max_steps = 300,
    num_map = 2,
    com_range = 10
    )
    robots = [model[i] for i in eachindex(model[1].plans)]
    extent = size(robots[1].state.space_state.gridmap)

    percent_of_map = zeros(length(robots)+1)
    astar_distances = zeros((length(robots), length(robots)))
    euclidean_distances = zeros((length(robots), length(robots)))
    
    percent_of_map[end] = count(x->any(x), [abmproperties(model).seen_all_gridmap[i,j,:] for i in 1:extent[1] for j in 1:extent[2]])/(extent[1]*extent[2]-abmproperties(model).nb_obstacles)
    df = DataFrame("nb_steps" => nb_step, "percent_of_map_all" => percent_of_map[end])

    for robot in robots

        percent_of_map[robot.id] = count(x->x, [abmproperties(model).seen_all_gridmap[i,j,robot.id] for i in 1:extent[1] for j in 1:extent[2]])/(extent[1]*extent[2]-abmproperties(model).nb_obstacles)

        for robot_prime in filter(x->x.id!=robot.id,robots[robot.id+1:end])
            astar_distances[robot.id, robot_prime.id] = length(plan_route!(robot, robot_prime.pos, pathfinder))
            astar_distances[robot_prime.id, robot.id] = astar_distances[robot.id, robot_prime.id]
            euclidean_distances[robot.id, robot_prime.id] = sqrt((robot.pos[1]-robot_prime.pos[1])^2 + (robot.pos[2]-robot_prime.pos[2])^2)
            euclidean_distances[robot_prime.id, robot.id] = euclidean_distances[robot.id, robot_prime.id]
        end

        # df = innerjoin(df, DataFrame("nb_steps" => robots[1].state.nb_coups, "percent_of_map_$(robot.id)" => percent_of_map[robot.id], "astar_distances_$(robot.id)" => [astar_distances[robot.id,:]], "euclidean_distances_$(robot.id)" =>[euclidean_distances[robot.id,:]], "seen_gridmap_$(robot.id)" => [abmproperties(model).seen_all_gridmap[:,:,robot.id]]), on = "nb_steps")
        
    end

    # write_header = false
    # if  robots[1].nb_step == 1
    #     write_header = true
    # end
    # CSV.write("Logs/nummap$(num_map)_extent$(extent1)$(extent2)/$(N).csv", df, delim = ';', header = write_header, append=true)

end
