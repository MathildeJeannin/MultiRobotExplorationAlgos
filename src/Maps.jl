function add_map(model, map, nb_robots)
    D = 2
    f = open("./src/maps/map$map.txt", "r")
    id = nb_robots+1
    _ = readline(f)
    invisible_cells = parse(Int64, readline(f))
    for line in readlines(f)
        str_pos = split(line, "\t")
        pos = (parse(Int64, str_pos[1]),parse(Int64, str_pos[2]))
        agent = Obstacle{D}(id, pos)
        add_agent!(agent, pos, model)
        id+=1
    end
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
    count = 0
    attempts = 0
    ids = nb_robots

    invisible_cells = 0 
    nb_obstacles = 0 

    while count < N && attempts < 100
        attempts += 1
        obstacle_width = rand(min_obstacle_size:max_obstacle_size)
        obstacle_height = rand(min_obstacle_size:max_obstacle_size)

        x = rand(2:extent[1] - obstacle_width - 1)
        y = rand(2:extent[2] - obstacle_height - 1)

        if is_valid_position(model, extent, x, y, obstacle_width, obstacle_height)
            count += 1
            invisible_cells += (obstacle_width - 1)*(obstacle_height - 1)
            for i in x:x+obstacle_width
                for j in y:y+obstacle_height
                    ids += 1
                    agent = Obstacle{2}(ids, (i,j))
                    add_agent!(agent, (i,j), model)
                    nb_obstacles += 1 
                end
            end
        end
    end
    return invisible_cells, nb_obstacles
end
   

function is_valid_position(model, extent, x, y, width, height)
    x_min = max(5, x-1)
    y_min = max(5, y-1)
    x_max = min(extent[1], x + width + 1)
    y_max = min(extent[2], y + height + 1)

    for i in x_min:x_max
        for j in y_min:y_max
            if length(ids_in_position((i,j),model)) > 0
                return false
            end
        end
    end

    return true
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
