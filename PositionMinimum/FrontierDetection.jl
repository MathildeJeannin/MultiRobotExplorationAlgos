function frontierDetection(pos::Tuple{Int,Int}, gridmap::MMatrix)
    queue = Tuple{Int,Int}[]
    frontier_cells = Set() 
    visited = Set()
    extent = size(gridmap)

    push!(queue, pos)

    while !isempty(queue)
        cell = x,y = queue[1]
        deleteat!(queue,1)
        push!(visited, cell)

        frontier_cells_local = isFrontier(cell, gridmap)
        for cell in frontier_cells_local
            push!(frontier_cells, cell)
        end

        for (dx,dy) in [(-1, 0), (1, 0), (0, -1), (0, 1), 
            (-1, -1), (-1, 1), (1, -1), (1, 1)]
            n = i,j = (x+dx,y+dy)
            if n ∉ visited && i > 0 && i <= extent[1] && j > 0 && j <= extent[2] && gridmap[i,j] == 0
                push!(queue, n)
            end
        end
        # _print_scan(queue,  (20,20))
    end
    # return frontier_cells
    return frontierRepartition(gridmap, frontier_cells)
end



function isFrontier(f::Tuple, gridmap::MMatrix)
    x,y = f
    extent = size(gridmap)
    frontier_cells = []
    if x > 0 && x <= extent[1] && y > 0 && y <= extent[2] && gridmap[x,y] == 0
        for (dx,dy) in [(-1, 0), (1, 0), (0, -1), (0, 1), 
            (-1, -1), (-1, 1), (1, -1), (1, 1)]
            a = (x+dx,y+dy)
            if a[1] > 0 && a[1] <= extent[1] && a[2] > 0 && a[2] <= extent[2] && gridmap[a[1],a[2]] == -2
                push!(frontier_cells, a)
            end
        end
    end
    return frontier_cells
end


function frontierRepartition(gridmap::MMatrix, frontier_cells::Set)
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
    queue = copy(frontier_cells)
    for cell in queue
        graph[cell] = Set()
        for cell_prime in queue
            if abs(cell[1]-cell_prime[1]) <= 1 && abs(cell[2] - cell_prime[2]) <= 1 && cell != cell_prime
                push!(graph[cell], cell_prime)
            end
        end
    end
    return graph
end
