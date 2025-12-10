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
extent1 = parse(Int64, ARGS[13])
extent2 = parse(Int64, ARGS[14])
nb_blocs = parse(Int64, ARGS[15])
begin_zone = parse(Int64, ARGS[16])

N = parse(Int64, ARGS[17])

t0 = now()

folder = "./expes/Logs/CenMCTS/num_map=$num_map,k_action=$(k_action)/"

file = folder*"$(N)_$(t0).csv"

# if num_map == -2
#     num_map = "./src/maps/random_indoor_maps/map$(N).txt"


try
    mkdir(folder)
catch e
end


nb_steps = run(vis_tree=false, vis_figure = false, show_progress = false, alpha_state=alpha_state, k_state=k_state, alpha_action=alpha_action, k_action=k_action, exploration_constant=exploration_constant,n_iterations=n_iterations, keep_tree=keep_tree, discount=discount, nb_robots=nb_robots, depth=depth, max_steps=max_steps, num_map=num_map, file=folder, id_expe=N, extent=(extent1,extent2), nb_blocs=nb_blocs, begin_zone=(begin_zone,begin_zone))


log_file = open(folder*"$N.txt", "a")
write(log_file, "run $(N); time = $(t0); nb_steps = $(nb_steps)\n")
close(log_file)


