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


function gridmap_update!(gridmap::MMatrix, id::Int, pos::Tuple, other_robots_poses::Vector, vis_range::Int, obstacles_poses::Vector, model::StandardABM; seen_gridmap = nothing, transition = false, rng = nothing, distribution = nothing)
    extent = size(gridmap)
    scan = collect(nearby_positions(pos, model, vis_range))
    lscan = limit_scan(scan, pos)
    rays = [raytracing(pos, l, vis_range) for l in lscan]

    gridmap[pos[1], pos[2]] = 0


    for ray in rays
        for element in ray[2:end]
            x,y = element[1],element[2]

            if transition && gridmap[x,y] == -2
                gridmap[x,y] = rand(rng, distribution)
                if gridmap[x,y] == -1
                    push!(obstacles_poses, (x,y))
                    break
                end
                seen_gridmap[x,y] = id
            end

            if gridmap[x,y] == -1 || (x,y) ∈ other_robots_poses
                break
            
            elseif gridmap[x,y] == -2
                if (x,y) ∈ obstacles_poses
                    gridmap[x,y] = -1
                    break
                else
                    gridmap[x,y] = 0
                end
            end
        end
    end

    # println("transition = $transition")
    # _print_gridmap(gridmap, [pos, other_robots_poses[1]])
    # sleep(0.1)
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
            elseif gridmap[nx,ny] == -2 && (nx,ny) ∉ already_seen
                push!(already_seen, (nx,ny))
                push!(queue, (nx,ny))
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
    for i in 1:extent[1]
        for j in 1:extent[2]
            surrounded, obstacle = is_surrounded(gridmap, (i,j))
            if surrounded
                for pos in obstacle
                    gridmap[pos[1],pos[2]] = -1
                end
            end
        end
    end
end


function raytracing(A, B, length_AB)
    compteur = 0
    xA,yA = A[1], A[2]
    xB,yB = B[1], B[2]
    ray = [(xA,yA)]

    while compteur < length_AB
        compteur+=1

        x = round(Int8,(compteur*xB + (length_AB-compteur)*xA)/length_AB)
        y = round(Int8,(compteur*yB + (length_AB -compteur)*yA)/length_AB)

        if (x,y) ∉ ray
            push!(ray, (x,y))
        end
    end

    return ray
end

function nearby_robots(pos::Tuple, model::ABM, r::Int)
    return filter(obj->(obj.pos in nearby_positions(pos,model,r) && !obj.isObstacle), collect(allagents(model)))
end

function nearby_robots(agent, model::ABM, r::Int)
    return nearby_robots(agent.pos, model, r)
end

function nearby_obstacles(pos::Tuple, model::ABM, r::Int)
    return filter(obj->(obj.pos in nearby_positions(pos,model,r) && obj.isObstacle), collect(allagents(model)))
end

function nearby_obstacles(agent, model::ABM, r::Int)
    return nearby_obstacles(agent.pos, model, r)
end



function _print_scan(scan_list, extent)
    for i in 1:extent[1]
        for j in 1:extent[2]
            if (i,j) ∈ scan_list 
                # || ([i,j] ∈ scan_list)
                print("x ")
            else
                print("_ ")
            end
            if j == extent[2]
                println()
            end
        end
    end
end


function _print_gridmap(gridmap, robots)
    println()
    pos = [r.pos for r in robots]
    extent = size(gridmap)
    for j in extent[2]:-1:1
        for i in 1:extent[1]
            if (i,j) in pos
                id = findall(item->item.pos == (i,j), robots)[1]
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