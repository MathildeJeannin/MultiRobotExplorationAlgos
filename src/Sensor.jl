function limit_scan(scan, center)
    push!(scan, center)
    sort!(scan)
    save = []
    for (i,s) in enumerate(scan)
        x,y = s
        if (x-1,y) ∉ scan || (x+1,y) ∉ scan || (x,y-1) ∉ scan || (x,y+1) ∉ scan
            push!(save, (x,y))
        end
    end
    return save
end


function gridmap_update!(gridmap::MMatrix, known_cells::Int64, id::Int, robots_pos::Union{Vector,SizedVector}, vis_range::Int, obstacles_pos::Vector, model::StandardABM; seen_cells = -1, transition = false, distribution = nothing)

    pos = robots_pos[id]
    other_robots_pos = robots_pos[1:end .!= id, :]

    extent = size(gridmap)
    scan = collect(nearby_positions(pos, model, vis_range))
    lscan = limit_scan(scan, pos)
    rays = [raytracing(pos, l, vis_range) for l in lscan]

    seen = Set()

    if gridmap[pos[1], pos[2]] == -2
        gridmap[pos[1], pos[2]] = 0
        if seen_cells != -1
            seen_cells += 1
        end
        known_cells += 1
        if !transition
            push!(seen, pos)
        end
    end

    for ray in rays
        for element in ray[2:end]
            x,y = element[1],element[2]

            if transition && gridmap[x,y] == -2
                gridmap[x,y] = rand(distribution)
                if seen_cells != -1
                    seen_cells += 1
                end
                known_cells += 1
                if gridmap[x,y] == -1
                    push!(obstacles_pos, (x,y))
                    break
                end
            end

            if gridmap[x,y] == -1 || (x,y) ∈ other_robots_pos
                break

            elseif gridmap[x,y] == -2
                if seen_cells != -1
                    seen_cells += 1
                end
                known_cells += 1
                if (x,y) ∈ obstacles_pos
                    gridmap[x,y] = -1
                    break
                else
                    gridmap[x,y] = 0
                end
            end
            if !transition
                push!(seen, (x,y))
            end
        end
    end

    if !transition
        for cell in seen
            abmproperties(model).seen_all_gridmap[id][cell[1],cell[2]] += 1
        end
    end
    return known_cells, seen_cells
end

function compute_new_pos(gridmap::MMatrix, id::Int, robots_pos::Union{Vector, SizedVector}, distance_to_move::Int, action::Tuple; goal = nothing)
    pos = robots_pos[id]
    other_robots_pos = robots_pos[1:end .!= id, :]

    extent = size(gridmap)

    if !isnothing(goal) 
        ray = raytracing(pos, goal, distance_to_move)
    else
        ray = raytracing(pos, (distance_to_move .* action) .+ pos, distance_to_move)
    end
    x,y = ray[1][1],ray[1][2]

    visible_robots = nearby_robots(pos, model, distance_to_move)

    if length(ray)==1
        return (x,y), (0,0)
    
    else
        for element in ray[2:end]
            x_prev, y_prev = x,y 
            x,y = element

            if (x > extent[1]) || (y > extent[2]) || (x < 1) || (y < 1)
                return (x_prev, y_prev), (0,0)
            end 

            if gridmap[x,y] == -1 
                return (x_prev, y_prev), (x,y)
            elseif (x,y) in other_robots_pos || (x,y) in visible_robots
                return (x_prev,y_prev), (0,0)
            end 
        end
    end
    return (x,y), (0,0)
end


function nearby_robots(pos::Tuple, model::ABM, r::Int)
    return filter(obj->(obj.pos in nearby_positions(pos,model,r) && typeof(obj) != Obstacle{2}), [model[i] for i in 1:abmproperties(model).nb_robots])
end

function nearby_robots(agent, model::ABM, r::Int)
    return nearby_robots(agent.pos, model, r)
end

function nearby_obstacles(pos::Tuple, model::ABM, r::Int)
    return filter(obj->(obj.pos in nearby_positions(pos,model,r) && typeof(obj) == Obstacle{2}), [model[i] for i in (abmproperties(model).nb_robots+1):(abmproperties(model).nb_obstacles[1]+abmproperties(model).nb_robots)])
end

function nearby_obstacles(agent, model::ABM, r::Int)
    return nearby_obstacles(agent.pos, model, r)
end


function raytracing(A, B, length_AB)
    compteur = 0
    xA,yA = A[1], A[2]
    xB,yB = B[1], B[2]
    ray = [(xA,yA)]

    while compteur < length_AB
        compteur+=1

        x = round(Int64,(compteur*xB + (length_AB-compteur)*xA)/length_AB)
        y = round(Int64,(compteur*yB + (length_AB-compteur)*yA)/length_AB)

        if (x,y) ∉ ray
            push!(ray, (x,y))
        end
    end

    return ray
end


function limitScanWithObstacles(id::Int, pos::Tuple, vis_range::Int, all_robots_pos::Union{Vector,SizedVector}, gridmap::MMatrix, scan::Vector)
    lscan = limit_scan(scan, pos)
    scan_accurate = Set()
    for l in lscan
        cell, _  = compute_new_pos(gridmap, id, all_robots_pos, vis_range, (0,0), goal = l)
        push!(scan_accurate, cell)
    end
    return scan_accurate
end



function pathfinder_update!(pathfinder::Any, gridmap::MMatrix)
    extent = X,Y = size(gridmap)
    for i in 1:X
        for j in 1:Y
            if gridmap[i,j] == -1
                pathfinder.walkmap[i,j] = false
            end
        end
    end
end


function _print_scan(scan_list, extent)
    for j in extent[2]:-1:1
        for i in 1:extent[1]
            if (i,j) ∈ scan_list 
                # || ([i,j] ∈ scan_list)
                print("x ")
            else
                print("_ ")
            end
            if i == extent[1]
                println()
            end
        end
    end
end


function distance(cell1,cell2)
    return sqrt((cell1[1] - cell2[1])^2 + (cell1[2] - cell2[2])^2)
end


function maxCellInScan(vis_range::Int, model::StandardABM)
    extent = abmproperties(model).extent
    pos = (round(extent[1]/2), round(extent[2]/2))
    return length(Set(nearby_positions(pos, model, vis_range)))
end


function _print_gridmap(gridmap, states)
    println()
    pos = [s.pos for s in states]
    extent = size(gridmap)
    for j in extent[2]:-1:1
        for i in 1:extent[1]
            if (i,j) in pos
                id = findall(item->item.pos == (i,j), states)[1]
                print("$id ")
            elseif gridmap[i,j] == -2
                print("x ")
            elseif gridmap[i,j] == -1 
                print("o ")
            elseif gridmap[i,j] == 0
                print("_ ")
            end
            if i == extent[1]
                println()
            end
        end
    end
    println("\n")
end


function _print_sequence(s::Vector{Vector{Tuple{String, Int64}}})
    _print_gridmap(r.state.space_state.gridmap,r.state.space_state.robots_plans[r.id].state.pos)
    for t in s
        index = t[2]
        if t[1]=='s'
            _print_gridmap(r.planner.tree.s_labels[index].space_state.gridmap, r.planner.tree.s_labels[index].space_state.robots_plans[r.id].state.pos)
        else
            println(r.planner.tree.a_labels[index])
        end 
    end
end

function _print_sequence(sequence::Vector{Int64}, r::Robot)
    _print_gridmap(r.state.space_state.gridmap, r.state.space_state.robots_plans[r.id].state.pos)
    for a in sequence
        println("action = $(r.planner.tree.a_labels[a])")
        next_s = r.planner.tree.transitions[a][1][1]
        println(next_s)
        _print_gridmap(r.planner.tree.s_labels[next_s].space_state.gridmap, r.planner.tree.s_labels[next_s].space_state.robots_plans[r.id].state.pos)
    end
end


function _print_seen_gridmap(seen_gridmap)
    println()
    extent = size(seen_gridmap)
    for j in extent[2]:-1:1
        for i in 1:extent[1]
            if seen_gridmap[i,j] == 0
                print("_ ")
            else
                print("$(seen_gridmap[i,j]) ")
            end
            # if gridmap[i,j] == 
            #     print("x ")
            # elseif gridmap[i,j] == -1 
            #     print("o ")
            # elseif gridmap[i,j] == 0
            #     print("_ ")
            # end
            if i == extent[1]
                println()
            end
        end
    end
    println("\n")
end



function is_surrounded(gridmap, pos)
    extent = size(gridmap)
    already_seen = []
    queue = []

    function boucle(pos)
        directions = [(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1),(-1,0)]
        x,y = pos
        surrounded = 0
        extent = size(gridmap)

        if !isempty(queue) && pos == queue[1]
            filter!(x->x!=pos, queue)
        end

        for (dx,dy) in directions
            nx = x+dx
            ny = y+dy
            if nx > extent[1] || ny > extent[2] || nx < 1 || ny < 1
            elseif gridmap[nx,ny] == 0 
                return false
            elseif gridmap[nx,ny] == -1
            elseif gridmap[nx,ny] == -2
                if (nx,ny) ∉ already_seen
                    push!(already_seen, (nx,ny))
                    push!(queue, (nx,ny))
                end
            end
        end

        if isempty(queue) 
            return true
        end

        return boucle(queue[1])
    end

    return boucle(pos), already_seen

end


function check_for_invisible_obstacles!(gridmap)
    extent = size(gridmap)
    output = 0
    for i in 1:extent[1]
        for j in 1:extent[2]
            if gridmap[i,j] == -2
                surrounded, obstacle = is_surrounded(gridmap, (i,j))
                if surrounded
                    output += length(obstacle)
                    for pos in obstacle
                        gridmap[pos[1],pos[2]] = -1
                    end
                end
            end
        end
    end
    return output
end


function _print_coverage_map(coverage_map::Union{MVector,Vector}, mode::String)
    nb_robots = length(coverage_map)
    max_value = maximum([maximum(coverage_map[i]) for i in 1:nb_robots])
    # max_value = 20
    a = round(Int64, sqrt(nb_robots))
    b = ceil(Int64, nb_robots/a)
    x = 1

    f = Figure()

    for i in 1:a
        for j in 1:b
            if x <= nb_robots
                ax = Axis(f[i,j])
                heatmap!(ax, Matrix(coverage_map[x]), colorrange = (0,max_value))
                x+=1
            end
        end
    end
    Colorbar(f[end,end+1], colorrange = (0,max_value))
    supertitle = Label(f[0,:], mode)
    display(f)
    # save(mode*"_map3_meancov.png", f)
end


function mean_cov(coverage_map)
    extent = size(coverage_map[1])
    output = Matrix{Float64}(zeros(extent[1],extent[2]))
    for i in 1:extent[1]
        for j in 1:extent[2]
            output[i,j] = mean(x[i,j] for x in coverage_map)
        end
    end
    return output
end