function frontierDetection(pos::Tuple{Int,Int}, gridmap::MMatrix)
    queue = Tuple{Int,Int}[]
    frontier_cells = Set() 
    visited = Set()

    push!(queue, pos)

    while !isempty(queue)
        cell = x,y = queue[1]
        deleteat!(queue,1)
        push!(visited, cell)

        if isFrontier(cell, gridmap)
            push!(frontier_cells, cell)
        end

        for (dx,dy) in [(-1, 0), (1, 0), (0, -1), (0, 1), 
            (-1, -1), (-1, 1), (1, -1), (1, 1)]
            n = i,j = (x+dx,y+dy)
            if n ∉ visited && i > 0 && i <= extent[1] && j > 0 && j <= extent[2] && gridmap[i,j] == 0
                push!(queue, n)
            end
        end
    end
    # return frontier_cells
    return kosaraju(gridmap, frontier_cells)
end



function isFrontier(f::Tuple, gridmap::MMatrix)
    x,y = f
    extent = size(gridmap)
    if x > 0 && x <= extent[1] && y > 0 && y <= extent[2] && gridmap[x,y] == 0
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




# function kosaraju(gridmap::MMatrix, frontier_cells::Set)
#     graph = buildGraph(gridmap, frontier_cells)
#     stack = []
#     visited = Set()

#     for node in keys(graph)
#         if node ∉ visited
#             dfs_first_pass!(node, graph, visited, stack)
#         end
#     end

#     transposed_graph = transposeGraph(graph)

#     empty!(visited)

#     frontiers = []
#     while !isempty(stack)
#         node = stack[1]
#         deleteat!(stack,1)
#         if node ∉ visited
#             component = []
#             dfs_second_pass!(node, transposed_graph, visited, component)
#             push!(frontiers, component)
#         end
#     end

#     return frontiers
# end


# function buildGraph(gridmap::MMatrix, frontier_cells::Set)
#     extent = size(gridmap)
#     graph = Dict()

#     for i in 1:extent[1]
#         for j in 1:extent[2]
#             node = (i,j)
#             if node ∈ frontier_cells
#                 graph[node] = []
#                 for (di,dj) in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
#                     ni, nj = i+di, j+dj
#                     if ni > 0 && ni <= extent[1] && nj > 0 && nj <= extent[2]
#                         if isFrontier((ni,nj), gridmap)
#                             push!(graph[node], (ni,nj))
#                         end
#                     end
#                 end
#             end
#         end
#     end
#     return graph
# end


# function transposeGraph(graph)
#     transposed_graph = Dict() 
#     for node in keys(graph)
#         for neighbour in graph[node]
#             if neighbour ∉ keys(transposed_graph)
#                 transposed_graph[neighbour] = []
#             end
#             push!(transposed_graph[neighbour], node)
#         end
#     end
#     return transposed_graph
# end


# function dfs_first_pass!(node, graph, visited, stack)
#     push!(visited, node)
#     for neighbour in graph[node]
#         if neighbour ∉ visited
#             dfs_first_pass!(neighbour, graph, visited, stack)
#         end
#     end
#     push!(stack, node)
# end


# function dfs_second_pass!(node, transposed_graph, visited, component)
#     push!(visited, node)
#     push!(component, node)
#     for neighbour in transposed_graph[node]
#         if neighbour ∉ visited
#             dfs_second_pass!(neighbour, transposed_graph, visited, component)
#         end
#     end
# end