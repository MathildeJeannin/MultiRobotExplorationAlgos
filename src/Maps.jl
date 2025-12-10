function add_map(model, map_path, nb_robots)
    D = 2
    f = open(map_path, "r")
    id = nb_robots+1
    extent_str = readline(f)
    extent_str_tuple = split(extent_str, ";")
    extent = (parse(Int64, extent_str_tuple[1]), parse(Int64, extent_str_tuple[2]))
    invisible_cells = parse(Int64, readline(f))
    gridmap = MMatrix{extent[1],extent[2]}(Int64.(-2*ones(Int64, extent)))
    for line in readlines(f)
        str_pos = split(line, "\t")
        pos = x,y = (parse(Int64, str_pos[1]),parse(Int64, str_pos[2]))
        agent = Obstacle{D}(id, pos)
        add_agent!(agent, pos, model)
        id+=1
        gridmap[x,y] = -1
    end
    return gridmap
end


function add_obstacles(model, nb_robots; N = 5, extent = (20,20))
    D = length(extent)
    T1 = Tuple(i for i in 1:extent[1])
    T2 = Tuple(i for i in 1:extent[2])
    obstacles = []
    for i in 1:N
        pos = (rand(T1),rand(T2))
        id = nb_robots + i
        agent = Obstacle{D}(id, pos)
        add_agent!(agent, pos, model)
    end
end


function add_simple_obstacles(model, extent, nb_robots; N = 1, min_obstacle_size = 4, max_obstacle_size = 7)
    invisible_cells, nb_obstacles, walls = generate_random_outdoor_map(extent; N=N, min_obstacle_size=min_obstacle_size, max_obstacle_size=max_obstacle_size)
    ids = nb_robots
    for cell in walls
        ids += 1
        agent = Obstacle{2}(ids, (cell[1],cell[2]))
        add_agent!(agent, (cell[1],cell[2]), model)
    end
    return invisible_cells, nb_obstacles
end


function generate_random_outdoor_map(extent; N = 1, min_obstacle_size = 4, max_obstacle_size = 7)
    count = 0
    attempts = 0
    invisible_cells = 0 
    nb_obstacles = 0 

    walls = Set()

    while count < N && attempts < 100
        attempts += 1
        obstacle_width = rand(min_obstacle_size:max_obstacle_size)
        obstacle_height = rand(min_obstacle_size:max_obstacle_size)

        x = rand(2:extent[1] - obstacle_width - 1)
        y = rand(2:extent[2] - obstacle_height - 1)

        if is_valid_position(walls, extent, x, y, obstacle_width, obstacle_height)
            count += 1
            invisible_cells += (obstacle_width - 1)*(obstacle_height - 1)
            for i in x:x+obstacle_width
                for j in y:y+obstacle_height
                    push!(walls, (i,j))
                    nb_obstacles += 1 
                end
            end
        end
    end
    return invisible_cells, nb_obstacles, walls
end


function save_random_outdoor_map(map_index, extent, nb_robots; N = 1, min_obstacle_size = 4, max_obstacle_size = 7)
    invisible_cells, _ ,walls = generate_random_outdoor_map(extent; N = N, min_obstacle_size = min_obstacle_size, max_obstacle_size = max_obstacle_size)
    f = open("./src/maps/random_outdoor_maps/map$(map_index).txt", "w")
    write(f, "$(extent[1]);$(extent[2])\n")
    write(f, "$(invisible_cells)\n")
    for cell in walls
        write(f, "$(cell[1])\t$(cell[2])\n")
    end
    close(f)
end

   

function is_valid_position(walls, extent, x, y, width, height)
    x_min = max(5, x-1)
    y_min = max(5, y-1)
    x_max = min(extent[1], x + width + 1)
    y_max = min(extent[2], y + height + 1)

    for i in x_min:x_max
        for j in y_min:y_max
            if (i,j) ∈ walls 
                return false
            end
        end
    end

    return true
end


function create_random_indoor_map(model, extent, nb_robots, min_room, max_room)
    walls = generate_random_indoor_map(extent, min_room, max_room)
    i = 1
    for w in walls
        id = nb_robots + i
        agent = Obstacle{2}(id, w)
        add_agent!(agent, w, model)
        i+=1
    end
    return length(walls)
end


function save_random_indoor_map(extent, min_room, max_room, map_index)
    walls =  generate_random_indoor_map(extent, min_room, max_room)
    f = open("./src/maps/random_indoor_maps/map$(map_index).txt", "w")
    write(f, "$(extent[1]);$(extent[2])\n")
    write(f, "0\n")
    for cell in walls
        write(f, "$(cell[1])\t$(cell[2])\n")
    end
    close(f)
end



function generate_random_indoor_map(extent::Tuple, min_room::Int64, max_room::Int64)
    root = Node((1,extent[1]),(1,extent[2]), rand(false:true), extent[1]*extent[2])
    queue = [root]
    nb_rooms = rand(min_room:max_room)

    count = 1
    attempts = 0

    walls = Set()

    while count <= nb_rooms && attempts < 500 && !isempty(queue)

        current_room = pick_node_with_priority(queue)

        horizontal = !current_room.horizontal
        horizontal ? (range_min, range_max) = current_room.wall_x : (range_min, range_max) = current_room.wall_y
        horizontal ? wall_range = current_room.wall_y : wall_range = current_room.wall_x
        corners = draw_corners(range_min, range_max)

        if corners != -1
            add_walls!(walls, corners, wall_range, horizontal) #
            nodes = create_nodes_from_corners(corners, wall_range, horizontal)
            for node in nodes
                push!(queue, node)
                count+=1
            end
        end
        attempts += 1

    end
    return walls
end


function pick_node_with_priority(queue::Vector{Node})
    index_max = 1
    node_max = queue[index_max]
    prio_max = node_max.priority
    for (i,node) in enumerate(queue)
        if node.priority > prio_max
            prio_max = node.priority
            node_max = node
            index_max = i
        end
    end
    deleteat!(queue, index_max)
    return node_max
end


function draw_corners(range_min::Int, range_max::Int)
    max_rooms_possible = Int64(floor((range_max-range_min)/6))
    if max_rooms_possible <= 1
        return -1
    end
    nb_rooms = rand(2:min(3,max_rooms_possible))
    corners = []
    while !is_valid_corners(corners, nb_rooms)
        corners = [range_min]
        for r in 1:nb_rooms-1
            c = rand(range_min:range_max)
            push!(corners, c)
        end
        push!(corners, range_max)
        sort!(corners)
    end
    return corners
end


function is_valid_corners(corners::Vector, nb_rooms::Int)
    if length(corners) != nb_rooms+1
        return false
    end
    for i in 2:nb_rooms+1
        if corners[i] - corners[i-1] < 4
            return false
        end
    end
    return true
end
      

function add_walls!(walls::Set, corners::Vector, wall_range::Tuple, horizontal::Bool)
    n = length(corners)
    for i in 2:n-1 # corners inclut debut et fin donc mur qui existe deja
        door = add_door(wall_range)
        for pos in wall_range[1]:wall_range[2]
            if pos ∉ door
                horizontal ? push!(walls, (corners[i], pos)) : push!(walls, (pos, corners[i]))
            end
        end
    end
end



function add_door(wall_range)
    D = rand(wall_range[1]+1:wall_range[2]-1)
    return [D-1,D,D+1]
end


function create_nodes_from_corners(corners::Vector, wall_range::Tuple, horizontal::Bool)
    n = length(corners)
    nodes = Node[]
    for i in 2:n
        horizontal ? new_node = Node((corners[i-1], corners[i]), wall_range, horizontal, (corners[i]-corners[i-1]+1)*(wall_range[2]-wall_range[1]+1)) : new_node = Node(wall_range, (corners[i-1], corners[i]), horizontal, (corners[i]-corners[i-1]+1)*(wall_range[2]-wall_range[1]+1))
        push!(nodes, new_node)
    end
    return nodes
end
    

function _set_to_gridmap(walls::Set, extent::Tuple)
    gridmap = MMatrix{extent[1],extent[2]}(Int64.(zeros(Int64, extent)))
    for cell in walls
        if cell[1] > 0 && cell[1] < extent[1]+1 && cell[2] > 0 && cell[2] < extent[2]+1 
            gridmap[cell[1],cell[2]] = -1
        end
    end
    return gridmap
end


function _test_generation_random_map(extent::Tuple, min_room::Int64, max_room::Int64)
    walls = generate_random_indoor_map(extent, min_room, max_room)
    gridmap = _set_to_gridmap(walls, extent)
    _print_gridmap(gridmap, [])
end

function _test_generation_random_map(extent::Tuple, N::Int64, min_obstacle_size::Int64, max_obstacle_size::Int64)
    _,_,walls = generate_random_outdoor_map(extent; N = N, min_obstacle_size = min_obstacle_size, max_obstacle_size = max_obstacle_size)
    gridmap = _set_to_gridmap(walls, extent)
    _print_gridmap(gridmap, [])
end


function create_map(size)

    global obstacles = []

    GLMakie.activate!()
    points = Observable(Point2f[])

    scene = Scene(camera = campixel!)

    mult = 25

    X = [0,size[1]*mult,size[1]*mult,0,0]
    Y = [0,0,size[2]*mult,size[2]*mult,0]

    lines!(scene, X,Y, color = :red, linewidth=mult)

    first_press = true
    x1,y1 = 0,0
  
    on(events(scene).mousebutton) do event
        if event.button == Mouse.left && event.action == Mouse.press
            mp = events(scene).mouseposition[]
            global x2,y2 = floor(Int64, mp[1]/mult), floor(Int64, mp[2]/mult)
            if first_press == true
                first_press = false
                x1,y1 = x2,y2
                println("first press, x=$x1, y=$y1")
                sleep(0.5)
            else
                lines!(scene, [x1*mult,x2*mult], [y1*mult,y2*mult], color = :black, linewidth=mult)
                first_press = true
                new_obstacles = raytracing([x1,y1],[x2,y2],sqrt((x1-x2)^2+(y1-y2)^2))
                for obs in new_obstacles
                    push!(obstacles, obs)
                end
            end
        end
    end

    on(events(scene).mouseposition) do mp
        mb = events(scene).mousebutton[]
        println("x = $(floor(Int64, mp[1]/mult)), y = $(floor(Int64, mp[2]/mult))")
    end

    on(events(scene).keyboardbutton) do event
        if event.key == Keyboard.q
            writedlm("temp.txt", size)
            writedlm("temp.txt", obstacles)
        end
    end

    scene

end
