using CSV
using DataFrames
using Dates

include("../CenMCTS/Run.jl")

alpha_state = parse(Float64, ARGS[1])
k_state = parse(Float64, ARGS[2])
alpha_action = parse(Float64, ARGS[3])
k_action = parse(Float64, ARGS[4])
exploration_constant = parse(Float64, ARGS[5])
n_iterations = parse(Int64, ARGS[6])
keep_tree = parse(Bool, ARGS[7])
discount = parse(Float64, ARGS[8])
nb_robots = parse(Int64, ARGS[9])
depth = parse(Int64, ARGS[10])
max_steps = parse(Int64, ARGS[11])
num_map = parse(Int64, ARGS[12])

N = parse(Int64, ARGS[13])

t0 = now()

folder = "/alpha_state=$alpha_state,k_state=$k_state,alpha_action=$alpha_action,k_action=$k_action,exploration_constant=$exploration_constant,n_iterations=$n_iterations,keep_tree=$keep_tree,discount=$discount,nb_robots=$nb_robots,depth=$depth,max_steps=$max_steps,num_map=$num_map,without_frontier/"

file = folder*"$(N)_$(t0).csv"



try
    mkdir("Resultats/Cen"*folder)
catch e
end
try
    mkdir("Logs/Cen"*folder)
catch e
end


nb_steps, cov = run(vis_tree=false, vis_figure = false, show_progress = false, alpha_state=alpha_state, k_state=k_state, alpha_action=alpha_action, k_action=k_action, exploration_constant=exploration_constant,n_iterations=n_iterations, keep_tree=keep_tree, discount=discount, nb_robots=nb_robots, depth=depth, max_steps=max_steps, num_map=num_map, file="Logs/Cen"*folder, id_expe=N)

# list_pos = Vector{Vector{robot_state}}(undef, length(log))
# list_gridmap = Vector{MMatrix}(undef, length(log))

# for (i,element) in enumerate(log)
#     list_pos[i] = element[1]
#     list_gridmap[i] = element[2]
# end

function coverage(cov::BitArray{3})
    extent = size(cov)
    nb_robots = extent[3]
    nb_cell_vues = 0
    gridmap_cov = MMatrix{extent[1],extent[2]}(zeros(extent[1],extent[2]))
    for i in 1:extent[1]
        for j in 1:extent[2]
            for ri in 1:nb_robots
                if cov[i,j,ri]
                    gridmap_cov[i,j] += 1
                    nb_cell_vues += 1
                end
            end
        end
    end

    nb_cell = extent[1]*extent[2]
    for i in 1:extent[1]
        for j in 1:extent[2]
            if gridmap_cov[i,j] == 0
                nb_cell -= 1
            end
        end
    end
    
    nb_cell_vues = nb_cell_vues/nb_cell
    return nb_cell_vues
end

nb_cell_vues = coverage(cov)

df = DataFrame(alpha_state=alpha_state, k_state=k_state, alpha_action=alpha_action, k_action=k_action, exploration_constant=exploration_constant,n_iterations=n_iterations, keep_tree=keep_tree, discount=discount, nb_robots=nb_robots, depth=depth, max_steps=max_steps, num_map=num_map, nb_steps = nb_steps, nb_cell_vues=nb_cell_vues)

CSV.write("Resultats/Cen"*file, df, writeheader=true, delim = ';', append=true)

log_file = open("Resultats/Cen/log_julia.txt", "a")
write(log_file, "run $(N); time = $(t0); alpha_state=$alpha_state, k_state=$k_state, alpha_action=$alpha_action, k_action=$k_action, exploration_constant=$exploration_constant,n_iterations=$n_iterations, keep_tree=$keep_tree, discount=$discount, nb_robots=$nb_robots, depth=$depth, max_steps=$max_steps, num_map=$num_map, nb_steps = $(nb_steps), nb_cell_vues = $nb_cell_vues")
close(log_file)
