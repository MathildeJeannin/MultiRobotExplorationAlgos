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
proba_communication = parse(Float64, ARGS[19])
fct_reward = ARGS[20]
fct_communication = ARGS[21]
filtering_info = parse(Bool, ARGS[22])
rollout = ARGS[23]

N = parse(Int64, ARGS[24])

t0 = now()

folder = "./expes/Logs/Dec/num_map=$(num_map),extent=$(extent1),nb_blocs=$(nb_blocs),rollout=$(rollout),alpha_state=$(alpha_state),exploration_constant=$(exploration_constant),reward=$(fct_reward),n_iterations=$(n_iterations),communication_range=$(com_range),proba_communication=$(proba_communication),data_relaying=$(fct_communication),filtering_info=$(filtering_info)/"


file = folder*"$(N)_$(t0).csv"


try
    mkdir(folder)
catch e
end


nb_steps = run(vis_tree=false, vis_figure = false, show_progress = false, alpha_state=alpha_state, k_state=k_state, alpha_action=alpha_action, k_action=k_action, exploration_constant=exploration_constant,n_iterations=n_iterations, keep_tree=keep_tree, discount=discount, nb_robots=nb_robots, depth=depth, max_steps=max_steps, num_map=num_map, com_range=com_range, alpha=alpha, file=folder, id_expe=N, extent=(extent1,extent2), nb_blocs=nb_blocs, begin_zone=(begin_zone,begin_zone), proba_communication=proba_communication, fct_reward=fct_reward, fct_communication=fct_communication , filtering_info=filtering_info, rollout=rollout)


log_file = open(folder*"$N.txt", "a")
write(log_file, "run $(N); time = $(t0); nb_steps = $(nb_steps)\n")
close(log_file)


