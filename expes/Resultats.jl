using CSV
using DataFrames
using GLMakie
using Statistics
using StaticArrays

function assemble()
    nb_robots=5
    res_file = "allres.csv"
    CSV.write(res_file, DataFrame(a = "num_map", b = "com_range", c = "fr_communication", d = "fct_reward", e = "fct_communication", f = "filtering_info", nb_steps = "nb_steps", cov = "cov"), delim = ";", header= false)

    mode = "Dec"
    path_folders = "./Logs/"*mode*"/"
    # path_folders = "./Resultats/"*mode*"/"
    folders = filter(x->x!="save", readdir(path_folders))
    for folder in folders
        string_param = split(folder, ['=',','])
        param = DataFrame(num_map = [parse(Int64, string_param[4]) for i in 1:nb_robots], com_range = [parse(Int64, string_param[6]) for i in 1:nb_robots], fr_communication = [parse(Float64, string_param[8]) for i in 1:nb_robots], fct_reward = [string_param[10] for i in 1:nb_robots], fct_communication = [string_param[12] for i in 1:nb_robots], filtering_info = [parse(Bool, string_param[14]) for i in 1:nb_robots])
    
        path = path_folders*folder*"/"
        files = filter(endswith("csv"), readdir(path))
        for file in files
            tmp_df = CSV.read(path*"/"*file, DataFrame, delim = ";")
            # res_df = DataFrame(mode = mode, alpha_action = tmp_df[1,:alpha_action], exploration_constant = tmp_df[1,:exploration_constant], nb_steps = tmp_df[1,:nb_steps], cov = [r[:cov] for r in eachrow(tmp_df)])
            res_df = deepcopy(param)
            res_df[!,:nb_steps] = [tmp_df[end,:nb_steps] for i in 1:nb_robots]
            res_df[!,:cov] = [tmp_df[end,"seen_gridmap_$i"] for i in 1:nb_robots]
            CSV.write(res_file, res_df, header=false, delim = ";",append=true)
        end
    end
end



function graph_evolution_during_simu(parameters_lines::Vector, title::String, label::Vector{String})
    df_param = CSV.read("parametersDec.csv", DataFrame, delim = ",")
    df = CSV.read("allres.csv", DataFrame, delim = ";")
    nb_robots=5
    coef_diviseur = 0 
    for i in 1:nb_robots-1
        coef_diviseur += i
    end
        
    parameters = df_param[parameters_lines,:]
    parameters = parameters[:, ["num_map", "extent1", "extent2", "com_range", "fr_communication", "fct_reward", "fct_communication", "filtering_info"]]

    all_res = Dict{DataFrameRow, Vector{DataFrame}}()
    

    for param in eachrow(parameters)
        path = "./Logs/Dec/n_iterations=500,num_map=$(param.num_map),com_range=$(param.com_range),fr_communication=$(param.fr_communication),fct_reward=$(param.fct_reward),fct_communication=$(param.fct_communication),filtering_info=$(Bool(param.filtering_info))/"
        files = filter(endswith("csv"), readdir(path))

        df_percent_all = DataFrame(steps = [i for i in 1:500])
        df_percent_robots = DataFrame(steps = [i for i in 1:500])
        df_astar = DataFrame(steps = [i for i in 1:500])
        df_euclidean = DataFrame(steps = [i for i in 1:500])
        df_nb_steps = DataFrame(nb_steps = [i for i in 1:500])

        for (i,file) in enumerate(files)

            df_percent_all[!,"simu$i"] = zeros(500)
            df_percent_robots[!,"simu$i"] = zeros(500)
            df_astar[!,"simu$i"] = zeros(500)
            df_euclidean[!,"simu$i"] = zeros(500)

            tmp_df = CSV.read(path*file, DataFrame, delim = ";")

            df_nb_steps[i,:nb_steps] = countlines(path*file) - 1

            s_percent_all = zeros(500)
            s_percent = zeros(500)
            s_astar = zeros(500)
            s_euclidean = zeros(500)
            for j in 1:500
                if j <= nrow(tmp_df)
                    s_percent_all[j] = tmp_df[j, :percent_of_map_all]

                    tmp_astar = []
                    tmp_euclidean = []
                    for r1 in 1:nb_robots
                        s_percent[j] += tmp_df[j, "percent_of_map_$r1"]/nb_robots
                        push!(tmp_astar, unpack_vector(tmp_df[j,"astar_distances_$r1"]))
                        push!(tmp_euclidean, unpack_vector(tmp_df[j,"euclidean_distances_$r1"]))
                        for r2 in r1+1:nb_robots
                            s_astar[j] += tmp_astar[r1][r2]/coef_diviseur
                            s_euclidean[j] += tmp_euclidean[r1][r2]/coef_diviseur
                        end
                    end
                else
                    s_percent_all[j] = s_percent_all[j-1]
                    s_percent[j] = s_percent[j-1]
                    s_astar[j] = s_astar[j-1]
                    s_euclidean[j] = s_euclidean[j-1]
                end
            end
            
            df_percent_all[!, "simu$i"] = s_percent_all
            df_percent_robots[!,"simu$i"] = s_percent
            df_astar[!,"simu$i"] = s_astar
            df_euclidean[!,"simu$i"] = s_euclidean
        end

    all_res[param] = [df_percent_all, df_percent_robots, df_astar, df_euclidean]
    end

    f = Figure()
    xlabel = "Number of steps"
    ax = [Axis(f[1,1], xlabel = xlabel, ylabel = "Percentage of map visited"), Axis(f[1,2], xlabel = xlabel, ylabel = "Mean percentage of map visited per robot"), Axis(f[2,1], xlabel = xlabel, ylabel = "Mean A* distance"), Axis(f[2,2], xlabel = xlabel, ylabel = "Mean euclidesn distance")]
    colors = [:red, :blue, :green , :orange, :black]


    for param in eachrow(parameters)
        percent_total = zeros(500)
        percent_robots = zeros(500)
        astar = zeros(500)
        euclidean = zeros(500)

        for i in 1:500
            percent_total[i] = mean(all_res[param][1][i,Between(:simu1,:simu50)])
            percent_robots[i] = mean(all_res[param][2][i,Between(:simu1,:simu50)])
            astar[i] = mean(all_res[param][3][i,Between(:simu1,:simu50)])
            euclidean[i] = mean(all_res[param][4][i,Between(:simu1,:simu50)])
        end
        k = rownumber(param)
        scatter!(ax[1], (1:500), percent_total, color = colors[k], label = label[k])
        scatter!(ax[2], (1:500), percent_robots, color = colors[k])
        scatter!(ax[3], (1:500), astar, color = colors[k])
        scatter!(ax[4], (1:500), euclidean, color = colors[k])
        hlines!(ax[4], [param.com_range], color = colors[k])
        
    end
    ylims!(ax[1], 0.0, 1.1)
    ylims!(ax[2], 0.0, 1.1)
    ylims!(ax[3], 0.0, 25.0)
    ylims!(ax[4], 0.0, 25.0)

    f[:,3] = Legend(f, ax[1], title, framevisible = false)
    # display(f)
    return f,ax
end





function whiskers_graph(parameters_lines::Vector, title::String, X_labels::Vector)
    # ligne dans le fichier excel 
    # param de la forme DataFrame("num_map", "com_range", "fr_communication", "fct_reward", "fct_communication", "filtering_info")

    # whiskers plot : 
    # 5 quantiles
    # la boite = toutes les donnes entre q2 et q4
    # moustache de taille : range*1.5 = (q4-q2)*1.5 = limit
    # en bas : min(limit, q2-min(data))
    # en haut :  min(limit, max(data)-q4)
    df_param = CSV.read("parametersDec.csv", DataFrame, delim = ",")
    df = CSV.read("allres.csv", DataFrame, delim = ";")
    nb_robots=5

    parameters = df_param[parameters_lines,:]
    parameters = parameters[:, ["num_map", "com_range", "fr_communication", "fct_reward", "fct_communication", "filtering_info"]]
    # sort!(parameters)

    f = Figure()
    # X = ["0.1", "0.5", "1.0", "5.0"]
    # Y_labels = ["$(100*i)" for i in 1:5]
    ax = Axis(f[1,1], xticks = 1:length(X_labels), xtickformat = A -> [X_labels[Int64(a)] for a in A], title = title, yticks=(0:100:500), xticklabelrotation = 0.0, ylabel = "Number of steps")
    ylims!(ax, -10, 510)
    data = []
    param_name = []
    j = 1
    for param in eachrow(parameters)
        resultats = df[(df.num_map .== param.num_map) .&& (df.com_range .== param.com_range) .&& (df.fr_communication .== param.fr_communication) .&& (df.fct_reward .== param.fct_reward) .&& (df.fct_communication .== param.fct_communication) .&& (df.filtering_info .== param.filtering_info),:]
        for i in 1:nrow(resultats)
            if i%nb_robots==0
                push!(data, resultats[i,:nb_steps])
                push!(param_name, j)
            end
        end
        j+=1
    end
    boxplot!(ax, param_name, data)
    return f,ax
end


function stats_map_random(parameters_lines::Vector)
    df_param = CSV.read("parametersDec.csv", DataFrame, delim = ",")
    df = CSV.read("allres.csv", DataFrame, delim = ";")
    nb_robots=5

    res_file = "stats_maps_random.csv"
    res_df = DataFrame(num_map = "num_map", com_range = "com_range", fr_communication = "fr_communication", fct_reward = "fct_reward", fct_communication = "fct_communication", filtering_info = "filtering_info", mean_cell_viewed = "mean_cell_viewed", mean_robots_saw = "mean_robots_saw", mean_cell_viewed_continuesly = "mean_cell_viewed_continuesly")
    CSV.write(res_file, res_df, delim = ";", header = false)
        
    parameters = df_param[parameters_lines,:]
    parameters = parameters[:, ["num_map", "extent1", "extent2", "com_range", "fr_communication", "fct_reward", "fct_communication", "filtering_info"]]

    # mean_robots_saw = MMatrix{extent[1],extent[2]}(Float64.(zeros(Float64, extent))) #moyenne nombre de robots qui a vu chaque cellule
    # mean_cell_viewed = MMatrix{extent[1],extent[2]}(Float64.(zeros(Float64, extent))) #moyenne nombre fois ou une cellule a ete vue
    mean_robots_saw = zeros(length(parameters_lines))
    mean_cell_viewed = zeros(length(parameters_lines))
    mean_cell_viewed_continuesly = zeros(length(parameters_lines))

    for param in eachrow(parameters)
        resultats = df[(df.num_map .== param.num_map) .&& (df.com_range .== param.com_range) .&& (df.fr_communication .== param.fr_communication) .&& (df.fct_reward .== param.fct_reward) .&& (df.fct_communication .== param.fct_communication) .&& (df.filtering_info .== param.filtering_info),:]
        extent = (param[:extent1], param[:extent2])
        for i in 1:5:nrow(resultats)
            cov_maps = [unpack_gridmap(resultats[i+l,:cov], extent) for l in 0:nb_robots-1]
            nb_cell_invisible = 0 
            tmp_cells = 0
            tmp_robots = 0
            for x in 1:extent[1]
                for y in 1:extent[2]
                    obs = true
                    for l in 1:nb_robots
                        if cov_maps[l][x,y] > 0 
                            obs = false
                            tmp_cells += cov_maps[l][x,y]
                            tmp_robots += 1
                        end
                    end
                    if obs
                        nb_cell_invisible += 1
                    end
                end
            end
            mean_cell_viewed[rownumber(param)] += tmp_cells/((extent[1]*extent[2] - nb_cell_invisible)*(nrow(resultats)/nb_robots))
            mean_robots_saw[rownumber(param)] += nb_robots*tmp_robots/((extent[1]*extent[2] - nb_cell_invisible)*nrow(resultats))
        end

        path = "./Logs/Dec/n_iterations=500,num_map=$(param.num_map),com_range=$(param.com_range),fr_communication=$(param.fr_communication),fct_reward=$(param.fct_reward),fct_communication=$(param.fct_communication),filtering_info=$(Bool(param.filtering_info))/"
        files = filter(endswith("csv"), readdir(path))
        for file in files
            maps_continues = gridmap_continue(path*file, nb_robots)
            nb_cell_invisible = 0 
            tmp_cells_continue = 0
            for x in 1:extent[1]
                for y in 1:extent[2]
                    obs = true
                    for l in 1:nb_robots
                        if maps_continues[l][x,y] > 0
                            obs = false
                            tmp_cells_continue += maps_continues[l][x,y]
                            println(maps_continues[l][x,y])
                        end
                    end
                    if obs
                        nb_cell_invisible += 1
                    end
                end
            end
            mean_cell_viewed_continuesly[rownumber(param)] += tmp_cells_continue/((extent[1]*extent[2] - nb_cell_invisible)*length(files))
        end

        one_res_df =  DataFrame(num_map = param.num_map, com_range = param.com_range, fr_communication = param.fr_communication, fct_reward = param.fct_reward, fct_communication = param.fct_communication, filtering_info = param.filtering_info, mean_cell_viewed = mean_cell_viewed[rownumber(param)], mean_robots_saw = mean_robots_saw[rownumber(param)], mean_cell_viewed_continuesly = mean_cell_viewed_continuesly[rownumber(param)])    

        CSV.write(res_file, one_res_df, header=false, delim = ";", append = true)
    end


    return mean_cell_viewed, mean_robots_saw, mean_cell_viewed_continuesly
end


function stats_map_fixe(parameters_lines::Vector)
    df_param = CSV.read("parametersDec.csv", DataFrame, delim = ",")
    df = CSV.read("allres.csv", DataFrame, delim = ";")
    nb_robots=5
        
    res_file = "stats_maps_fixe.csv"
    res_df = DataFrame(num_map = "num_map", com_range = "com_range", fr_communication = "fr_communication", fct_reward = "fct_reward", fct_communication = "fct_communication", filtering_info = "filtering_info", mean_cell_viewed = "mean_cell_viewed", mean_robots_saw = "mean_robots_saw", mean_cell_viewed_continuesly = "mean_cell_viewed_continuesly")
    CSV.write(res_file, res_df, delim = ";", header = false)

    parameters = df_param[parameters_lines,:]
    parameters = parameters[:, ["num_map", "extent1", "extent2", "com_range", "fr_communication", "fct_reward", "fct_communication", "filtering_info"]]

    extent = (parameters[1,:extent1], parameters[1,:extent2])

    mean_robots_saw = [MMatrix{param[:extent1],param[:extent2]}(Float64.(zeros(Float64, (param[:extent1], param[:extent2])))) for param in eachrow(parameters)] #moyenne nombre de robots qui a vu chaque cellule
    mean_cell_viewed = [MMatrix{param[:extent1],param[:extent2]}(Float64.(zeros(Float64, (param[:extent1], param[:extent2])))) for param in eachrow(parameters)] #moyenne nombre fois ou une cellule a ete vue
    mean_cell_viewed_continuesly = [MMatrix{param[:extent1],param[:extent2]}(Float64.(zeros(Float64, (param[:extent1], param[:extent2])))) for param in eachrow(parameters)] #moyenne nombre fois ou une cellule a ete vue en continue

    for param in eachrow(parameters)
        resultats = df[(df.num_map .== param.num_map) .&& (df.com_range .== param.com_range) .&& (df.fr_communication .== param.fr_communication) .&& (df.fct_reward .== param.fct_reward) .&& (df.fct_communication .== param.fct_communication) .&& (df.filtering_info .== param.filtering_info),:]
        extent = (param[:extent1], param[:extent2])
        for i in 1:5:nrow(resultats)
            cov_maps = [unpack_gridmap(resultats[i+l,:cov], extent) for l in 0:nb_robots-1]
            tmp_cells = 0
            tmp_robots = 0
            for x in 1:extent[1]
                for y in 1:extent[2]
                    for l in 1:nb_robots
                        if cov_maps[l][x,y] > 0 
                            mean_cell_viewed[rownumber(param)][x,y] += cov_maps[l][x,y]/nrow(resultats)
                            mean_robots_saw[rownumber(param)][x,y] += nb_robots/nrow(resultats)
                        end
                    end
                end
            end
        end
        path = "./Logs/Dec/n_iterations=500,num_map=$(param.num_map),com_range=$(param.com_range),fr_communication=$(param.fr_communication),fct_reward=$(param.fct_reward),fct_communication=$(param.fct_communication),filtering_info=$(Bool(param.filtering_info))/"
        files = filter(endswith("csv"), readdir(path))
        for file in files
            maps_continues = gridmap_continue(path*file, nb_robots)
            for x in 1:extent[1]
                for y in 1:extent[2]
                    for l in 1:nb_robots
                        mean_cell_viewed_continuesly[rownumber(param)][x,y] += maps_continues[l][x,y]/(length(files)*nb_robots)
                    end
                end
            end
        end

        one_res_df =  DataFrame(num_map = param.num_map, com_range = param.com_range, fr_communication = param.fr_communication, fct_reward = param.fct_reward, fct_communication = param.fct_communication, filtering_info = param.filtering_info, mean_cell_viewed = [mean_cell_viewed[rownumber(param)]], mean_robots_saw = [mean_robots_saw[rownumber(param)]], mean_cell_viewed_continuesly = [mean_cell_viewed_continuesly[rownumber(param)]])    

        CSV.write(res_file, one_res_df, header=false, delim = ";", append = true)

    end
    return mean_cell_viewed, mean_robots_saw, mean_cell_viewed_continuesly
end



function gridmap_continue(file::String, nb_robots::Int64, extent)
    gridmaps = [[MMatrix{extent[1],extent[2]}(zeros(extent[1],extent[2]))] for i in 1:nb_robots]
    output = [MMatrix{extent[1],extent[2]}(zeros(extent[1],extent[2])) for i in 1:nb_robots]
    tmp_df = CSV.read(file, DataFrame, delim = ";")

    prev_gridmap = [MMatrix{extent[1],extent[2]}(zeros(extent[1],extent[2])) for i in 1:nb_robots]
    next_gridmap = [unpack_gridmap(tmp_df[1,"seen_gridmap_$i"], (extent[1],extent[2])) for i in 1:nb_robots]
    for r in 1:nb_robots
        for x in 1:extent[1]
            for y in 1:extent[2]
                if next_gridmap[r][x,y] == 2
                    prev_gridmap[r][x,y] = 1
                end
            end
        end
        push!(gridmaps[r], prev_gridmap[r])
        push!(gridmaps[r], next_gridmap[r] .- gridmaps[r][end])
    end

    for i in 2:nrow(tmp_df)
        positions = unpack_positions(tmp_df[i,:positions])
        for r in 1:nb_robots
            next_gridmap[r] = unpack_gridmap(tmp_df[i,"seen_gridmap_$r"], (extent[1],extent[2]))
            prev_gridmap[r] = unpack_gridmap(tmp_df[i-1,"seen_gridmap_$r"], (extent[1],extent[2]))
            push!(gridmaps[r], next_gridmap[r] .- prev_gridmap[r])
            gridmaps[r][end][positions[r][1], positions[r][2]] = 1
        end

        
    end

    for r in 1:nb_robots
        for x in 1:extent[1]
            for y in 1:extent[2]
                for i in 2:nrow(tmp_df)
                    if gridmaps[r][i][x,y] == 1 && gridmaps[r][i-1][x,y] == 0
                        output[r][x,y] += 1
                    end
                end
            end
        end
    end
    return output
    # return gridmaps
end


function stats()

    df = CSV.read("allres.csv", DataFrame, delim = ";")
    compteur=1
    mode = "Dec"

    df_param = CSV.read("parameters$(mode).csv", DataFrame, delim = ",")
    df_res = copy(df_param)
    longueur = size(df_param)[1]
    df_res[!,:percent_success] = zeros(longueur)
    df_res[!,:mean_nb_steps] = zeros(longueur)
    df_res[!,:var_nb_steps] = zeros(longueur)
    df_res[!,:stand_var_nb_steps] = zeros(longueur)
    df_res[!,:min_nb_steps] = zeros(longueur)
    df_res[!,:max_nb_steps] = zeros(longueur)
    df_res[!,:median_nb_steps] = zeros(longueur)
    df_res[!,:mean_cov] = [zeros(df_param[1,:extent1],df_param[1,:extent2]) for i in 1:longueur]


    for param in eachrow(df_param[1:17,:])
        res = df[(df.num_map .== param.num_map) .&& (df.com_range .== param.com_range) .&& (df.fr_communication .== param.fr_communication) .&& (df.fct_reward .== param.fct_reward) .&& (df.fct_communication .== param.fct_communication) .&& (df.filtering_info .== param.filtering_info),:]
        row = rownumber(param)

        df_res[row,:mean_cov] = zeros(param[:extent1],param[:extent2])

        println(row)

        df_res[row,:percent_success] = count(x->x[:nb_steps]<param[:max_steps], collect(eachrow(res)))/nrow(res)
        df_res[row,:mean_nb_steps] = mean(res[!,:nb_steps])
        df_res[row,:var_nb_steps] = var(res[!,:nb_steps])
        df_res[row,:stand_var_nb_steps] = std(res[!,:nb_steps])
        df_res[row,:min_nb_steps] = minimum(res[!,:nb_steps])
        df_res[row,:max_nb_steps] = maximum(res[!,:nb_steps])
        df_res[row,:median_nb_steps] = median(res[!,:nb_steps])

        all_matrix = Vector{Matrix}(undef, nrow(res))
        for i in 1:nrow(res)
            r = res[i,:]
            all_matrix[i] = unpack_gridmap(r[:cov], tuple(param[:extent1],param[:extent2]))
        end

        for i in 1:param[:extent1]
            for j in 1:param[:extent2]
                df_res[row,:mean_cov][i,j] = mean(x[i,j] for x in all_matrix)
            end
        end
        if param[:num_map] > 0
            _print_coverage_map([df_res[row,:mean_cov]], mode, "map$(param[:num_map]),extent$(param[:extent1])")
        end
    end

    CSV.write("stats$mode.csv", df_res, delim = ";", append=true, header = true)
end


function graphics()

    df_stats = CSV.read("stats$mode.csv", DataFrame, delim = ";")

    statistic = ["mean_nb_steps", "stand_var_nb_steps", "min_nb_steps", "max_nb_steps", "median_nb_steps","percent_success"]
    # statistic = ["percent_success"]
    mode = ["Dec","Cen","DecPositionMinimum","CenPositionMinimum"]
    maps = [[2,20],[3,20],[4,40],[5,40],[-1,20],[-1,30],[-1,40]]
    colors = [:red, :blue, :yellow, :green, :purple]


    for (i,stat) in enumerate(statistic)
        # stat = "mean"
        f = Figure(size = (24*96/2.54,13.5*96/2.54))
        X = ["20x20 inside", "20x20 outsitde", "40x40 inside", "40x40 outsitde", "20x20 random", "30x30 random", "40x40 random"]
        if stat == "stand_var_nb_steps"
            yticks = 0:10:100
        elseif stat == "percent_success"
            yticks = 0:0.1:1.0
        else
           yticks = 0:100:500
        end
        ax = Axis(f[1,1], xticks = 1:7, xtickformat = A -> [X[Int64(a)] for a in A], title = stat, yticks=yticks)
        Y = [0.0 for i in 1:7]
        for (x,m) in enumerate(mode)
            df_stats = CSV.read("stats$m.csv", DataFrame, delim = ";")
            for (j,map) in enumerate(maps)
                for row in eachrow(df_stats)
                    if row[:num_map]==map[1] && row[:extent1]==map[2]
                        Y[j] = row[stat]
                        scatter!(ax, j+0.2*(x-1), Y[j], color = colors[x], label = m)
                    end
                end
            end
        end
        if stat == "stand_var_nb_steps"
            ylims!(ax, -10, 110)
        elseif stat == "percent_success"
            ylims!(ax, -0.1, 1.1)
        else
            ylims!(ax, -10, 510)
        end
        f[1,2] = leg = axislegend(ax, merge = true, unique = true)
        save("$(stat).png", f, px_per_unit=2.0) 
    end
   
end


function _print_coverage_map(coverage_map::Vector, mode::String, param::String)
    nb_robots = length(coverage_map)
    # max_value = maximum([maximum(coverage_map[i]) for i in 1:nb_robots])
    max_value = 10
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
    save(mode*"_"*param*"_meancov.png", f)
    # display(f)
end

function unpack_gridmap(cov::String, extent::Tuple)
    matrix = Matrix{Int}(zeros(extent[1],extent[2]))
    cov_string = split(chop(cov, head=1, tail=1), "; ")
    for j in 1:extent[1]
        matrix[j,:] = parse.(Int, split(cov_string[j], " "))
    end
    return matrix
end

function _print_coverage_map(coverage_map::Union{MVector,Vector})
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
    # supertitle = Label(f[0,:], mode)
    display(f)
    # save(mode*"_map3_meancov.png", f)
end


function unpack_vector(str)
    mots = split(str, ",")
    out = Vector{Float64}(undef, length(mots))
    for (i,mot) in enumerate(mots)
        if mot[1] == '[' 
            mot = split(mot, '[')[2]
        elseif mot[end] == ']'
            mot = split(mot, ']')[1]
        end
        out[i] = parse(Float64, mot)
    end
    return out
end

function unpack_positions(str)
    mots = split(str, ['(',')'])
    out = []
    for (i,mot) in enumerate(mots)
        if length(mot) > 2
            pos_str = split(mot, ',')
            push!(out, (parse(Int64, pos_str[1]), parse(Int64, pos_str[2])))
        end
    end
    return out
end


function figure_maps()
    f = Figure()
    axes = [Axis(f[i,j], aspect = DataAspect(), xgridvisible = false, ygridvisible = false, xticklabelsize = 10.0, yticklabelsize = 10) for i in 1:2 for j in 1:2]
    for m in 2:5
        file = open("./src/maps/map$m.txt", "r")
        size_str = split(readline(file), ";")
        extent = (parse(Int64, size_str[1]), parse(Int64, size_str[2]))
        gridmap = Matrix{Int64}(zeros(extent[1],extent[2]))
        readline(file)
        for line in readlines(file)
            cell_str = split(line, "\t")
            x,y = (parse(Int64, cell_str[1]), parse(Int64, cell_str[2]))
            gridmap[x,y] = -2
        end
        heatmap!(axes[m-1], gridmap, colorrange = (-2,0))
        ylims!(axes[m-1], 0, 40)
        xlims!(axes[m-1], 0, 40)
        hidespines!(axes[m-1])
        axes[m-1].xticks=(0:10:extent[1])
        axes[m-1].yticks=(0:10:extent[2])
    end
    # Colorbar(f[end,end+1], colorrange = (-2,0), ticklabelsize=10.0)
    # display(f)
    return f
end