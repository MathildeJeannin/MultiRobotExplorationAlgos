function frontierDetection(pos::Tuple{Int,Int}, scan::Vector, gridmap::MMatrix, frontier_cells::Vector)
    extent = size(gridmap)
    # At = scan = collect(nearby_positions(pos, model, vis_range))
    rayEndPoints = limit_scan(scan, pos)
    queue = vcat(rayEndPoints, filter(in(frontier_cells), scan))
    visited = Vector{Tuple{Int,Int}}(undef,0)

    while !isempty(queue)
        x,y = queue[1]
        deleteat!(queue, 1)

        isFrontierVar = false

        if isFrontier((x,y),gridmap)
            push!(frontier_cells, (x,y))
            isFrontierVar = true
        end

            
        if isFrontierVar || gridmap[x,y] == -1
            for (dx,dy) in [(-1, 0), (1, 0), (0, -1), (0, 1), 
                (-1, -1), (-1, 1), (1, -1), (1, 1)]
                a = (x+dx,y+dy)
                if a[1] > 0 && a[1] <= extent[1] && a[2] > 0 && a[2] <= extent[2]
                    if a ∉ visited && a ∈ scan && gridmap[a[1],a[2]] == 0
                        push!(visited, a)
                        push!(queue, a)
                    end
                end
            end
        end
    end

    toDelete = []
    for f in filter(in(frontier_cells), scan)
        if !isFrontier(f,gridmap)
            push!(toDelete, f)
        end
    end

    frontier_cells = filter(x->x ∉ toDelete, frontier_cells)

    return kosaraju(gridmap, frontier_cells)
end

function isFrontier(f::Tuple, gridmap::MMatrix)
    x,y = f
    if gridmap[x,y] == 0
        for (dx,dy) in [(-1, 0), (1, 0), (0, -1), (0, 1), 
            (-1, -1), (-1, 1), (1, -1), (1, 1)]
            a = (x+dx,y+dy)
            if a[1] > 0 && a[1] <= extent[1] && a[2] > 0 && a[2] <= extent[2] && gridmap[a[1],a[2]] == -2
                return true
            end
        end
    end
    return false
end

        
function kosaraju(gridmap::MMatrix, frontier_cells::Vector)
    graph = buildGraph(gridmap, frontier_cells)
    stack = []
    visited = Set()

    for node in keys(graph)
        if node ∉ visited
            dfs_first_pass!(node, graph, visited, stack)
        end
    end

    transposed_graph = transposeGraph(graph)

    empty!(visited)

    frontiers = [] 
    while !isempty(stack)
        node = stack[1]
        deleteat!(stack,1)
        if node ∉ visited
            component = []
            dfs_second_pass!(node, transposed_graph, visited, component)
            push!(frontiers, component)
        end
    end

    return frontiers
end


function buildGraph(gridmap::MMatrix, frontier_cells::Vector)
    extent = size(gridmap)
    graph = Dict()

    for i in 1:extent[1]
        for j in 1:extent[2]
            node = (i,j)
            if node ∈ frontier_cells
                graph[node] = []
                for (di,dj) in [(-1, 0), (1, 0), (0, -1), (0, 1), 
                    (-1, -1), (-1, 1), (1, -1), (1, 1)]
                    ni, nj = i+di, j+dj
                    if ni > 0 && ni <= extent[1] && nj > 0 && nj <= extent[2]
                        if isFrontier((ni,nj), gridmap)
                            push!(graph[node], (ni,nj))
                        end
                    end
                end
            end
        end
    end
    return graph
end


function transposeGraph(graph)
    transposed_graph = Dict() 
    for node in keys(graph)
        for neighbour in graph[node]
            if neighbour ∉ keys(transposed_graph)
                transposed_graph[neighbour] = []
            end
            push!(transposed_graph[neighbour], node)
        end
    end
    return transposed_graph
end


function dfs_first_pass!(node, graph, visited, stack)
    push!(visited, node)
    for neighbour in graph[node]
        if neighbour ∉ visited
            dfs_first_pass!(neighbour, graph, visited, stack)
        end
    end
    push!(stack, node)
end


function dfs_second_pass!(node, transposed_graph, visited, component)
    push!(visited, node)
    push!(component, node)
    for neighbour in transposed_graph[node]
        if neighbour ∉ visited
            dfs_second_pass!(neighbour, transposed_graph, visited, component)
        end
    end
end
