
function compute_actions_cenMCTS(nb_robots)
    theta = [i*pi/4 for i in 0:7]
    rad_actions = [(round(cos(θ),digits=2),round(sin(θ), digits=2)) for θ in theta]

    ids = collect(Int8(i) for i in 1:nb_robots)
    
    actionsNR = [Vector{Action}(undef,length(rad_actions)) for i in 1:nb_robots]

    for id in ids
        for index_a in eachindex(rad_actions)
            actionsNR[id][index_a] = Action(rad_actions[index_a])
        end
    end

    A = [actionsNR[i] for i in 1:nb_robots]
    all_actions = collect(Iterators.product(A...))


    global vector_all_actions = Vector{ActionCen}(undef, length(all_actions))
    for i in eachindex(all_actions)
        vector_all_actions[i] = ActionCen([j for j in all_actions[i]])
    end
    
    return vector_all_actions
end



function compute_actions_decMCTS()
    theta = [i*pi/4 for i in 0:7]
    # theta = [i*pi/2 for i in 0:3]
    rad_actions = [(round(cos(θ),digits=2),round(sin(θ), digits=2)) for θ in theta]
    possible_actions = Vector{Action}(undef,length(rad_actions))
    for (i,a) in enumerate(rad_actions)
        possible_actions[i] = Action(a)
    end

    return possible_actions
end