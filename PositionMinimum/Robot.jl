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
        invisible_cells,
        nb_robots
    )

    global model = AgentBasedModel(Union{RobotPosMin{D},Obstacle{D}}, space; agent_step!,
        scheduler = scheduler, 
        properties = properties
    )


    #obstacles
    if num_map == 0
        add_obstacles(model, nb_robots; N = nb_obstacles, extent = extent)
    elseif num_map > 0
        add_map(model, num_map, nb_robots)
    else 
        abmproperties(model).invisible_cells[1] = add_simple_obstacles(model, extent, nb_robots; N = 3)
    end

    theta = [i*pi/4 for i in 0:7]
    rad_actions = [(round(cos(θ),digits=2),round(sin(θ), digits=2)) for θ in theta]
    possible_actions = Vector{Action}(undef,length(rad_actions))
    for (i,a) in enumerate(rad_actions)
        possible_actions[i] = Action(a)
    end


    for n ∈ 1:nb_robots
        pos = (rand(T1),rand(T2))
        id = n
        isObstacle = false

        obstacles_poses = [element.pos for element in nearby_obstacles(pos, model, vis_range)]
    
        gridmap_n = copy(gridmap)

        known_cells, seen_cells = gridmap_update!(gridmap_n, 0, id, [(1,1) for i in 1:nb_robots], vis_range, obstacles_poses, model)

        walkmap = BitArray{2}(trues(extent))
        # obs = [r.pos for r in collect(nearby_obstacles((1,1),model,100))]
        # for p in obs
        #     walkmap[p[1],p[2]] = false
        # end
        pathfinder = Agents.Pathfinding.AStar(abmspace(model), walkmap=walkmap)

        robot = RobotPosMin{D}(id, pos, vis_range, com_range, gridmap_n, [(1,1) for i in 1:nb_robots], pathfinder, [], Set())
        add_agent!(robot, pos, model)

        for pos in robot.all_robots_pos
            if pos != robot.pos
                robot.pathfinder.walkmap[pos[1],pos[2]] = false
            end
        end
        robot.frontiers, all_frontiers = frontierDetection(robot.id, robot.pos, robot.vis_range, robot.gridmap, robot.all_robots_pos, robot.frontiers; need_repartition=true)
        goal = positionMinimum(all_frontiers, robot.gridmap, robot.all_robots_pos[Not(robot.id)], robot.pos)
        robot.plan = collect(plan_route!(robot, goal, robot.pathfinder))

        for pos in robot.all_robots_pos
            if pos != robot.pos
                robot.pathfinder.walkmap[pos[1],pos[2]] = true
            end
        end
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


function agent_step!(robot, model)
    extent = size(robot.gridmap)
    nb_robots = abmproperties(model).nb_robots

    in_range = nearby_robots(robot, model, robot.com_range)
    for r in in_range
        exchange_positions!(robot, r)
        merge_gridmaps!(robot,r)
        exchange_frontiers!(robot,r)
    end

    # for pos in robot.all_robots_pos
    #     robot.pathfinder.walkmap[pos[1],pos[2]] = true
    # end

    scan = collect(nearby_positions(robot.pos, model, robot.vis_range))
    
    while !isempty(robot.plan) && robot.plan[1] in scan
        action = (robot.plan[1][1]-robot.pos[1], robot.plan[1][2]-robot.pos[2])./distance(robot.plan[1], robot.pos)

        new_pos, _ = compute_new_pos(robot.gridmap, robot.id, robot.all_robots_pos, 1, action)
        move_agent!(robot, new_pos, model)
        robot.all_robots_pos[robot.id] = new_pos
        deleteat!(robot.plan,1)
    end        
    
    obstacles_pos = [element.pos for element in nearby_obstacles(robot, model, robot.vis_range)]
    gridmap_update!(robot.gridmap, 0, robot.id, robot.all_robots_pos, robot.vis_range, obstacles_pos, model)
    pathfinder_update!(robot.pathfinder, robot.gridmap)

    if count(x->x == -2, robot.gridmap) > abmproperties(model).invisible_cells[1]
        robot.frontiers, all_frontiers = frontierDetection(robot.id, robot.pos, robot.vis_range, robot.gridmap, robot.all_robots_pos, robot.frontiers; need_repartition=true)
        goal = positionMinimum(all_frontiers, robot.gridmap, robot.all_robots_pos[Not(robot.id)], robot.pos)
        robot.plan = collect(plan_route!(robot, goal, robot.pathfinder))
    end

end
