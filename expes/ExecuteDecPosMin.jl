using CSV
using DataFrames
using Dates

include("../DecPositionMinimum/Run.jl")

nb_robots = parse(Int64, ARGS[1])
max_steps = parse(Int64, ARGS[2])
map_path = ARGS[3]
extent1 = parse(Int64, ARGS[4])
extent2 = parse(Int64, ARGS[5])
nb_blocs = parse(Int64, ARGS[6])
begin_zone = parse(Int64, ARGS[7])

N = parse(Int64, ARGS[8])

t0 = now()

splited_path_to_map = split(map_path, "/")

if splited_path_to_map[4] == "first_maps"
    folder = "./expes/Logs/DecPosMin/type_of_map=$(splited_path_to_map[4]),num_map=$(split(splited_path_to_map[5], ".")[1][4:end])/"
else
    folder = "./expes/Logs/DecPosMin/type_of_map=$(splited_path_to_map[4])/"
    map_path = map_path*"map$(N).txt"
end

file = folder*"$(N)_$(t0).csv"


try
    mkdir(folder)
catch e
end


nb_steps = run(vis_figure = false, nb_robots=nb_robots, max_steps=max_steps, map_path=map_path, file=folder, id_expe=N, extent=(extent1,extent2), nb_blocs=nb_blocs, begin_zone=(begin_zone,begin_zone))

log_file = open(folder*"$N.txt", "a")
write(log_file, "run $(N); time = $(t0); nb_steps = $(nb_steps)\n")
close(log_file)
