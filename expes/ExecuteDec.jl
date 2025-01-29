using CSV
using DataFrames
using Dates

include("../DecMCTS/Run.jl")

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
com_range = parse(Int64, ARGS[13])
alpha = parse(Float64, ARGS[14])
extent1 = parse(Int64, ARGS[15])
extent2 = parse(Int64, ARGS[16])
nb_blocs = parse(Float64, ARGS[17])
begin_zone = parse(Int64, ARGS[18])
fr_communication = parse(Int64, ARGS[19])
fct_reward = ARGS[20]
fct_communication = ARGS[21]
use_old_info = parse(Bool, ARGS[22])

N = parse(Int64, ARGS[23])

t0 = now()

folder = "/fct_communication=$fct_communication/"

file = folder*"$(N)_$(t0).csv"


try
    mkdir("Resultats/Dec"*folder)
catch e
end
try
    mkdir("Logs/Dec"*folder)
catch e
end


nb_steps, cov = run(vis_tree=false, vis_figure = false, show_progress = false, alpha_state=alpha_state, k_state=k_state, alpha_action=alpha_action, k_action=k_action, exploration_constant=exploration_constant,n_iterations=n_iterations, keep_tree=keep_tree, discount=discount, nb_robots=nb_robots, depth=depth, max_steps=max_steps, num_map=num_map, com_range=com_range, alpha=alpha, file="Logs/Dec"*folder, id_expe=N, extent=(extent1,extent2), nb_blocs=nb_blocs, begin_zone=(begin_zone,begin_zone), fr_communication=fr_communication, fct_reward=fct_reward, fct_communication=fct_communication , use_old_info=use_old_info)

df = DataFrame(alpha_state=alpha_state, k_state=k_state, alpha_action=alpha_action, k_action=k_action, exploration_constant=exploration_constant,n_iterations=n_iterations, keep_tree=keep_tree, discount=discount, nb_robots=nb_robots, depth=depth, max_steps=max_steps, num_map=num_map, com_range=com_range, extent=(extent1,extent2), nb_blocs=nb_blocs, nb_communication=nb_communication, fct_reward=fct_reward, fct_communication=fct_communication, use_old_info=use_old_info, nb_steps = nb_steps, cov=cov)

CSV.write("Resultats/Dec"*file, df, writeheader=true, delim = ';', append=true)

log_file = open("Logs/Dec"*folder*"$N.txt", "a")
write(log_file, "run $(N); time = $(t0); nb_steps = $(nb_steps), cov=$cov\n")
close(log_file)


