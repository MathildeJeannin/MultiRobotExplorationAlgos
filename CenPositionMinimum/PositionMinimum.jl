function frontWave(frontier::Set, gridmap::MMatrix)
    extent = size(gridmap)
    champ_potentiel = -1*MMatrix{extent[1],extent[2]}(Int64.(ones(extent[1],extent[2])))

    f0 = frontierCenter(frontier)
    champ_potentiel[f0[1],f0[2]] = 0
    fprev = [f0]

    frontWaveRec!(fprev, champ_potentiel, gridmap, 1)
    return champ_potentiel
end


function frontWaveRec!(fprev::Vector, champ_potentiel::MMatrix, gridmap::MMatrix, i::Int)
    fnext = []
    extent = size(gridmap)
    for (x,y) in fprev
        for (di,dj) in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
            ni, nj = x+di, y+dj
            if ni > 0 && ni <= extent[1] && nj > 0 && nj <= extent[2] && gridmap[ni,nj] == 0 && champ_potentiel[ni,nj] == -1
                champ_potentiel[ni,nj] = i
                push!(fnext, (ni,nj))
            end
        end
    end
    if isempty(fnext)
        return 
    else
        frontWaveRec!(fnext, champ_potentiel, gridmap, i+1)
    end
end

function frontierCenter(frontier::Set)
    if length(frontier) == 1
        return first(frontier)
    end

    best_cell = nothing
    min_dist_sum = 10000000

    for cell in frontier
        S = 0 
        for other_cell in frontier
            S += distance(cell, other_cell)
        end
        if S < min_dist_sum
            best_cell = cell
            min_dist_sum = S
        end
    end

    return best_cell
end


function goToFrontier(frontier_cell::Tuple, pos::Tuple, gridmap::MMatrix)
    extent = size(gridmap)
    neighbours = []
    best_neighbour = frontier_cell
    min_dist = 1000000000

    for (dx,dy) in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        nx, ny = frontier_cell[1]+dx, frontier_cell[2]+dy
        if nx > 0 && nx <= extent[1] && ny > 0 && ny <= extent[2] && gridmap[nx,ny] == 0
            d = distance(pos, (nx,ny))
            if d < min_dist
                best_neighbour = (nx,ny)
                min_dist = d
            end
        end
    end

    return best_neighbour
end


function distance(cell1,cell2)
    return sqrt((cell1[1] - cell2[1])^2 + (cell1[2] - cell2[2])^2)
end



function positionMinimum(all_frontiers::Vector, gridmap::MMatrix, other_robots_pos::Vector, pos::Tuple)
    extent = size(gridmap)

    min_frontier = all_frontiers[1]
    min_cout = extent[1]*extent[2]

    for frontier in all_frontiers
        champ_potentiel = frontWave(frontier, gridmap)
        cout = champ_potentiel[pos[1],pos[2]]
        S = 0
        for other_pos in other_robots_pos
            if champ_potentiel[other_pos[1],other_pos[2]] < cout
                S+=1
            end
        end
        if S < min_cout
            min_cout = S
            min_frontier = frontier
        end
    end

    frontier_center = frontierCenter(min_frontier)
    return frontier_center
end