MCTS.node_tag(s::GWPos) = "[$(s[1]),$(s[2])]"
MCTS.node_tag(a::Symbol) = "[$a]"

function initialize_model(
    nb_robots,
    extent,
    nb_obstacles, 
    num_map;
    begin_zone = (1,1), 
    vis_range = 3.0,
    com_range = 2.0,
    invisible_cells = 0,
    nb_blocs = 3
)
    
    # initialize model
    space = GridSpace(extent, periodic = false, metric = :euclidean)
    scheduler = Schedulers.fastest  # they are executed 
    gridmap = MMatrix{extent[1],extent[2]}(Int64.(-2*ones(Int64, extent)))
    walkmap = BitArray{2}(trues(extent))
    plan = Vector{Vector{Tuple}}(undef, nb_robots)
    frontiers = Set()

    D = length(extent)

    T1 = Tuple(i for i in 1:begin_zone[1])
    T2 = Tuple(i for i in 1:begin_zone[2])

    invisible_cells = [invisible_cells]
    nb_obstacles = [nb_obstacles]

    properties = (
        seen_all_gridmap = MVector{nb_robots, MMatrix}(MMatrix{extent[1],extent[2]}(Int64.(zeros(Int64, extent))) for i in 1:nb_robots),
        nb_obstacles, 
        invisible_cells,
        nb_robots
        )

    global model = AgentBasedModel(Union{RobotCen{D},Obstacle{D}}, space; agent_step!,
        scheduler = scheduler, 
        properties = properties
    )

    pathfinder = Agents.Pathfinding.AStar(abmspace(model), walkmap=walkmap)
    global memory = SharedMemory(gridmap, frontiers, plan, pathfinder)

    #obstacles
    if num_map == 0
        add_obstacles(model, nb_robots; N = nb_obstacles[1], extent = extent)
    elseif num_map > 0
        add_map(model, num_map, nb_robots)
    else 
        abmproperties(model).invisible_cells[1], abmproperties(model).nb_obstacles[1] = add_simple_obstacles(model, extent, nb_robots; N = nb_blocs)
    end

    walkmap = BitArray{2}(trues(extent[1],extent[2]))
    pathfinder = Agents.Pathfinding.AStar(abmspace(model), walkmap=walkmap)

    pos = Vector{Tuple{Int,Int}}(undef, nb_robots)
    for n ∈ 1:nb_robots
        pos[n] = (rand(T1),rand(T2))
        while !isempty(ids_in_position(pos[n], model))
            pos[n] = (rand(T1),rand(T2))
        end
    end

    for n ∈ 1:nb_robots
        id = n

        robot = RobotCen{D}(id, pos[n], vis_range, pathfinder, Set())
        add_agent!(robot, pos[n], model) 
    end

    robots = [model[i] for i in 1:nb_robots]

    all_robots_pos = pos

    for robot in robots
        obstacles_pos = [element.pos for element in nearby_obstacles(robot, model, robot.vis_range)]

        gridmap_update!(memory.gridmap, 0, robot.id, all_robots_pos, robot.vis_range, obstacles_pos, model)

        memory.frontiers, all_frontiers = frontierDetection(robot.id, robot.pos, robot.vis_range, memory.gridmap, all_robots_pos,  memory.frontiers; need_repartition=true)

        goal = positionMinimum(all_frontiers, memory.gridmap, all_robots_pos[Not(robot.id)], robot.pos)
        memory.plan[robot.id] = collect(plan_route!(robot, goal, pathfinder))
    end

    return model
end


function agent_step!(model)
    extent = size(memory.gridmap)
    nb_robots = abmproperties(model).nb_robots
    robots = [model[i] for i in 1:nb_robots]

    all_robots_pos = [r.pos for r in robots]

    pathfinder = Agents.Pathfinding.AStar(abmspace(model), walkmap=memory.pathfinder.walkmap)
    
    for robot in robots
        scan = collect(nearby_positions(robot.pos, model, robot.vis_range))
        
        if !isempty(memory.plan[robot.id]) && !isempty(memory.plan[robot.id][1])
            action = (memory.plan[robot.id][1][1]-robot.pos[1], memory.plan[robot.id][1][2]-robot.pos[2])./distance(memory.plan[robot.id][1], robot.pos)
            deleteat!(memory.plan[robot.id],1)
        else
            action = rand([(1.0, 0.0),
            (0.71, 0.71),
            (0.0, 1.0),
            (-0.71, 0.71),
            (-1.0, 0.0),
            (-0.71, -0.71),
            (-0.0, -1.0),
            (0.71, -0.71)])
        end
 
        new_pos, _ = compute_new_pos(memory.gridmap, robot.id, all_robots_pos, 1, action)
        move_agent!(robot, new_pos, model)
        all_robots_pos[robot.id] = new_pos
        
        obstacles_pos = [element.pos for element in nearby_obstacles(robot, model, robot.vis_range)]
        gridmap_update!(memory.gridmap, 0, robot.id,all_robots_pos, robot.vis_range, obstacles_pos, model)
        pathfinder_update!(pathfinder, memory.gridmap)

        if count(x->x == -2, memory.gridmap) > abmproperties(model).invisible_cells[1]
            memory.frontiers, all_frontiers = frontierDetection(robot.id, robot.pos, robot.vis_range, memory.gridmap, all_robots_pos, memory.frontiers; need_repartition=true)
            goal = positionMinimum(all_frontiers, memory.gridmap, all_robots_pos[Not(robot.id)], robot.pos)
            memory.plan[robot.id] = collect(plan_route!(robot, goal, pathfinder))
        end
    end
end
