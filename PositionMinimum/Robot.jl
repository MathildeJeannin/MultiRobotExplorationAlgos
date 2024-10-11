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
        add_simple_obstacles(model, extent, nb_robots; N = 3)
    end

    theta = [i*pi/4 for i in 0:7]
    rad_actions = [(round(cos(θ),digits=2),round(sin(θ), digits=2)) for θ in theta]
    possible_actions = Vector{action_robot}(undef,length(rad_actions))
    for (i,a) in enumerate(rad_actions)
        possible_actions[i] = action_robot(a)
    end


    for n ∈ 1:nb_robots
        pos = (rand(T1),rand(T2))
        id = n
        isObstacle = false

        obstacles_poses = [element.pos for element in nearby_obstacles(pos, model, vis_range)]
    
        gridmap_n = copy(gridmap)

        known_cells, seen_cells = gridmap_update!(gridmap_n, 0, id, [(1,1) for i in 1:nb_robots], vis_range, obstacles_poses, model)

        walkmap = BitArray{2}(trues(extent))
        obs = [r.pos for r in collect(nearby_obstacles((1,1),model,100))]
        for p in obs
            walkmap[p[1],p[2]] = false
        end
        pathfinder = Agents.Pathfinding.AStar(abmspace(model), walkmap=walkmap)

        agent = RobotPosMin{D}(id, pos, vis_range, com_range, gridmap_n, [(1,1) for i in 1:nb_robots], pathfinder, [])
        add_agent!(agent, pos, model)
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


function compute_new_pos(gridmap::MMatrix, id::Int, robots_pos::Union{Vector, SizedVector}, vis_range::Int, action::Tuple)
    pos = robots_pos[id]
    other_robots_pos = robots_pos[1:end .!= id, :]

    extent = size(gridmap)
    ray = raytracing(pos, (vis_range .* action) .+ pos, vis_range)
    x,y = ray[1][1],ray[1][2]

    if length(ray)==1
        return (x,y), (0,0)
    
    else
        for element in ray[2:end]
            x_prev, y_prev = x,y 
            x,y = element[1], element[2]

            if (x > extent[1]) || (y > extent[2]) || (x < 1) || (y < 1)
                return (x_prev, y_prev), (0,0)
            end 

            if gridmap[x,y] == -1 
                return (x_prev, y_prev), (x,y)
            elseif (x,y) in other_robots_pos
                return (x_prev,y_prev), (0,0)
            end 
        end
    end
    return (x,y), (0,0)
end


function agent_step!(robot, model)
    extent = size(robot.gridmap)
    nb_robots = abmproperties(model).nb_robots

    println("robot $(robot.id)")

    if count(x->x == -2, robot.gridmap) >= abmproperties(model).invisible_cells

        println("communicating ...")

        in_range = nearby_robots(robot, model, robot.com_range)
        for r in in_range
            exchange_positions!(robot, r)
            merge_gridmaps!(robot,r)
        end


        if isempty(robot.plan)

            println("computing frontiers")
            all_frontiers = frontierDetection(robot.pos, robot.gridmap)
            goal = positionMinimum(all_frontiers, robot.gridmap, robot.all_robots_pos[Not(robot.id)], robot.pos)
            robot.plan = collect(plan_route!(robot, goal, robot.pathfinder))
        end

        scan = collect(nearby_positions(robot.pos, model, robot.vis_range))

        println("computing new position ...")
       
        while !isempty(robot.plan) && robot.plan[1] in scan
            action = (robot.plan[1][1]-robot.pos[1], robot.plan[1][2]-robot.pos[2])./distance(robot.plan[1], robot.pos)

            new_pos, _ = compute_new_pos(robot.gridmap, robot.id, robot.all_robots_pos, 1, action)
            move_agent!(robot, new_pos, model)
            robot.all_robots_pos[robot.id] = new_pos
            deleteat!(robot.plan,1)
        end

        println("moving ...")

        
        
        println("updating gridmap ...")

        obstacles_pos = [element.pos for element in nearby_obstacles(robot, model, robot.vis_range)]
        gridmap_update!(robot.gridmap, 0, robot.id, robot.all_robots_pos, robot.vis_range, obstacles_pos, model)

    end
end


# function defineAction(robot::RobotPosMin, scan::Vector)
#     i = 0
#     cells = []
#     in_scan = true
#     while in_scan && i < length(robot.plan)
#         i+=1
#         cell = robot.plan[i]
#         if cell ∈ scan
#             push!(cells, cell)
#         else
#             in_scan = false
#         end
#     end

#     new_pos = (0,0)
#     action = (0,0)
#     prev_pos = robot.pos
#     prev_action = (0,0)

#     pos_ok = true
#     k = 1

#     while pos_ok && !isempty(cells)

#         action = (cells[1][1]-robot.pos[1], cells[1][2]-robot.pos[2])./distance(cells[1], robot.pos)

#         new_pos, _ = compute_new_pos(robot.gridmap, robot.id, robot.all_robots_pos, robot.vis_range, action)

#         if new_pos == prev_pos
#             pos_ok = false
#         end 
        
#         deleteat!(robot.plan,1)
#         deleteat!(cells, 1)

#         prev_action = action

#     end

#     return prev_action
# end
