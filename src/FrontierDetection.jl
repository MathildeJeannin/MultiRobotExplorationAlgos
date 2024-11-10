# function frontierDetection(robot::Robot)
function frontierDetection(id::Int, pos::Tuple, vis_range::Int, gridmap::MMatrix, all_robots_pos::Union{Vector, SizedVector}, frontiers::Set; need_repartition=true)
    scan = collect(nearby_positions(pos, model, vis_range))
    lscan = limitScanWithObstacles(id, pos, vis_range, all_robots_pos, gridmap, scan)
    queue = vcat(collect(lscan), filter(in(frontiers), scan))
    visited = Tuple{Int,Int}[]
    while !isempty(queue)
        x,y = cell = queue[1]
        deleteat!(queue, 1)

        is_frontier = isFrontier(cell, gridmap)
        if gridmap[x,y] == 0 && is_frontier
            push!(frontiers, cell)
        end

        if is_frontier || gridmap[x,y] == -1
            for (dx,dy) in [(-1, 0), (1, 0), (0, -1), (0, 1), 
                (-1, -1), (-1, 1), (1, -1), (1, 1)]
                nx,ny = a = x+dx,y+dy
                if a ∉ visited && a in scan && gridmap[nx,ny] == 0
                    push!(visited, a)
                    push!(queue, a)
                end
            end
        end
    end
   
    for f in frontiers
        # _, is_frontier = isFrontier(f, gridmap)
        is_frontier = isFrontier(f,gridmap)
        if !is_frontier
            delete!(frontiers, f)
        end
    end
    if need_repartition
        return frontiers, frontierRepartition(gridmap, frontiers)
    else
        return frontiers
    end
end


function isFrontier(f::Tuple, gridmap::MMatrix)
    x,y = f
    extent = size(gridmap)
    frontier_cells = []
    is_frontier = false
    if x > 0 && x <= extent[1] && y > 0 && y <= extent[2] && gridmap[x,y] == 0
        for (dx,dy) in [(-1, 0), (1, 0), (0, -1), (0, 1), 
            (-1, -1), (-1, 1), (1, -1), (1, 1)]
            a = (x+dx,y+dy)
            if a[1] > 0 && a[1] <= extent[1] && a[2] > 0 && a[2] <= extent[2] && gridmap[a[1],a[2]] == -2
                # push!(frontier_cells, a)
                return true
            end
        end
    end
    # if !isempty(frontier_cells)
    #     is_frontier = true
    # end
    # return frontier_cells, is_frontier
    return false
end


function frontierRepartition(gridmap::MMatrix, frontier_cells::Set) # = BFS
    graph = buildGraph(frontier_cells)
    frontiers = []
    visited = Set()
    for node in keys(graph)
        if node ∉ visited
            queue = [node]
            push!(visited, node)
            component = Set([node])
            while !isempty(queue)
                for cell in graph[queue[1]]
                    if cell ∉ visited 
                        push!(component, cell)
                        push!(visited, cell)
                        push!(queue, cell)
                    end
                end
                deleteat!(queue, 1)
            end
            push!(frontiers, component)
        end
    end
    return frontiers
end


function buildGraph(frontier_cells::Set)
    graph = Dict()
    for cell in frontier_cells
        graph[cell] = Set()
        for cell_prime in frontier_cells
            if abs(cell[1]-cell_prime[1]) <= 1 && abs(cell[2] - cell_prime[2]) <= 1 && cell != cell_prime
                push!(graph[cell], cell_prime)
            end
        end
    end
    return graph
end
