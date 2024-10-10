using CSV
using DataFrames
using Dates

include("../../Run.jl")

num_map = parse(Int64, ARGS[1])
extent1 = parse(Int64, ARGS[2])
extent2 = parse(Int64, ARGS[3])
N = parse(Int64, ARGS[4])

extent = (extent1, extent2)

try 
    mkdir("Resultats/nummap$(num_map)_extent$(extent1)$(extent2)/")
    mkdir("Logs/nummap$(num_map)_extent$(extent1)$(extent2)/")
catch e 
end

t0 = now()

nb_steps = run(num_map = num_map, extent = extent, id_expe = N)

duration = now()-t0


df = DataFrame(num_map = num_map, extent = extent, nb_steps = nb_steps, duration = duration )


CSV.write("Resultats/nummap$(num_map)_extent$(extent1)$(extent2)/$(N).csv", df, delim = ';', append=true, header = true)


file = open("log_julia.txt", "a")
write(file, "num_map = $(num_map); extent = $(extent); run $(N); time = $(t0); nb_steps = $(nb_steps); duration = $(canonicalize(duration))\n")
close(file)
