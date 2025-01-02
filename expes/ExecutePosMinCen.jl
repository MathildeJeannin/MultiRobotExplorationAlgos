using CSV
using DataFrames
using Dates

include("../CenPositionMinimum/Run.jl")

nb_robots = parse(Int64, ARGS[1])
max_steps = parse(Int64, ARGS[2])
num_map = parse(Int64, ARGS[3])
extent1 = parse(Int64, ARGS[4])
extent2 = parse(Int64, ARGS[5])
nb_blocs = parse(Int64, ARGS[6])
begin_zone = parse(Int64, ARGS[7])

N = parse(Int64, ARGS[8])

t0 = now()

folder = "/num_map=$num_map,extent=$extent1/"

file = folder*"$(N)_$(t0).csv"



try
    mkdir("Resultats/CenPositionMinimum"*folder)
catch e
end
try
    mkdir("Logs/CenPositionMinimum"*folder)
catch e
end


nb_steps, cov = run(vis_figure = false, nb_robots=nb_robots, max_steps=max_steps, num_map=num_map, file="Logs/CenPositionMinimum"*folder, id_expe=N, extent=(extent1,extent2), nb_blocs=nb_blocs, begin_zone=(begin_zone,begin_zone))

df = DataFrame(nb_robots=nb_robots, max_steps=max_steps, num_map=num_map, extent=(extent1,extent2), nb_blocs=nb_blocs, begin_zone=begin_zone, nb_steps = nb_steps, cov=cov)

CSV.write("Resultats/CenPositionMinimum"*file, df, writeheader=true, delim = ';', append=true)

log_file = open("Logs/CenPositionMinimum"*folder*"$N.txt", "a")
write(log_file, "run $(N); time = $(t0); nb_steps = $(nb_steps), cov = $cov\n")
close(log_file)

