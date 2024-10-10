POMDPs.actions(m::robotMDP) = m.possible_actions


POMDPs.stateindex(m::robotMDP, s::mdp_state) = m.indices[s]
POMDPs.actionindex(m::robotMDP, a::Vector{action_robot}) = m.indices[a]

POMDPs.discount(m::robotMDP) = m.discount

function POMDPs.transition(m::robotMDP, s::mdp_state, a::Vector{action_robot})
    ImplicitDistribution() do x
        rng = MersenneTwister(rand(1:10^39))

        extent = size(s.gridmap)

        nb_obst_vus = count(x->x==-1, s.gridmap)
        nb_cell_vues = count(x->x!=-2, s.gridmap)

        # P_obstacle = (m.nb_obstacle - nb_obst_vus)/(extent[1]*extent[2] - nb_cell_vues)
        P_obstacle = 0

        next_robot_list = Vector{robot_state}(undef, length(s.robot_list))
        for robot in s.robot_list
            next_robot_list[robot.id] = robot_state(robot.id, robot.pos)
        end
        next_gridmap = deepcopy(s.gridmap)
        next_seen_gridmap = deepcopy(s.seen_gridmap)

        distribution = SparseCat([-1,0], [P_obstacle,1-P_obstacle])

        for i in eachindex(s.robot_list)
            other_robots_poses = [rob.pos for rob in next_robot_list if rob.id != s.robot_list[i].id]

            action = filter(x->x.id==s.robot_list[i].id, a)[1]

            next_pos, obstacle_pos = compute_new_pos!(next_gridmap, m.vis_range, s.robot_list[i].pos, other_robots_poses, action, transition = true, proba_obs = P_obstacle, rng = rng, distribution = distribution)
            next_robot_list[s.robot_list[i].id] = robot_state(s.robot_list[i].id, next_pos)

            gridmap_update!(next_gridmap, s.robot_list[i].id, next_pos, other_robots_poses, m.vis_range, [obstacle_pos], model, seen_gridmap = next_seen_gridmap, transition = true, rng = rng, distribution = distribution)

        end      

        # println(next_seen_gridmap)

        sp = mdp_state(next_robot_list, next_gridmap, next_seen_gridmap, s.nb_coups+1)
        return sp
    end
end


function POMDPs.reward(m::robotMDP, s::mdp_state, a::Vector{action_robot}, sp::mdp_state)
    nb_robots = length(s.robot_list)

    r = (count(x->x==-2, s.gridmap) - count(x->x==-2, sp.gridmap))
    
    return r
end


function POMDPs.isterminal(m::robotMDP, s::mdp_state)
    return count(i->i==-2, s.gridmap) == 0 
end


function compute_actions(nb_robots)
    theta = [i*pi/4 for i in 0:7]
    # theta = [i*pi/2 for i in 0:3]
    rad_actions = [(round(cos(θ),digits=2),round(sin(θ), digits=2)) for θ in theta]

    ids = collect(Int8(i) for i in 1:nb_robots)
    
    actionsNR = [Vector{action_robot}(undef,length(rad_actions)) for i in 1:nb_robots]

    for id in ids
        for index_a in eachindex(rad_actions)
            actionsNR[id][index_a] = action_robot(id, rad_actions[index_a])
        end
    end

    A = [actionsNR[i] for i in 1:nb_robots]
    all_actions = collect(Iterators.product(A...))


    global vector_all_actions = Vector{Vector{action_robot}}(undef, length(all_actions))
    for i in eachindex(all_actions)
        vector_all_actions[i] = [j for j in all_actions[i]]
    end
    
    return vector_all_actions
end
