POMDPs.actions(m::RobotMDP) = m.possible_actions


POMDPs.stateindex(m::RobotMDP, s::StateCen) = m.indices[s]
POMDPs.actionindex(m::RobotMDP, a::ActionCen) = m.indices[a]

POMDPs.discount(m::RobotMDP) = m.discount

function POMDPs.transition(m::RobotMDP, s::StateCen, a::ActionCen)
    ImplicitDistribution() do x
        extent = size(s.gridmap)

        nb_obst_vus = count(x->x==-1, s.gridmap)
        nb_cell_vues = count(x->x!=-2, s.gridmap)

        next_robots_states = Vector{RobotState}(undef, length(s.robots_states))
        for robot in s.robots_states
            next_robots_states[robot.id] = RobotState(robot.id, robot.pos)
        end
        next_gridmap = deepcopy(s.gridmap)
        next_seen = [0,0,0]
        # next_seen_gridmap = deepcopy(s.seen_gridmap)

        distribution = SparseCat([-1,0],[0,1.0])

        all_robots_pos = [r.pos for r in next_robots_states]
        for rs in s.robots_states

            action = a.directions[rs.id].direction

            next_pos, obstacle_pos = compute_new_pos(next_gridmap, rs.id, all_robots_pos, 1, action)
            next_robots_states[rs.id] = RobotState(rs.id, next_pos)
            all_robots_pos[rs.id] = next_pos

            _, next_seen[rs.id] = gridmap_update!(next_gridmap, 0, rs.id, all_robots_pos, m.vis_range, [obstacle_pos], model, transition = true, distribution = distribution)
        end      

        sp = StateCen(next_gridmap, next_robots_states, next_seen, s.nb_coups+1)
        return sp
    end
end


function POMDPs.reward(m::RobotMDP, s::StateCen, a::ActionCen, sp::StateCen)
    nb_robots = length(s.robots_states)

    # r = (count(x->x==-2, s.gridmap) - count(x->x==-2, sp.gridmap)) + count(x->x!=0, sp.seen)
    r = (count(x->x==-2, s.gridmap) - count(x->x==-2, sp.gridmap))
    
    return r
end


function POMDPs.isterminal(m::RobotMDP, s::StateCen)
    return count(i->i==-2, s.gridmap) == 0 
end


struct FrontierPolicy{M<:RobotMDP} <: Policy 
    mdp::M
end


function POMDPs.action(rollout_policy::FrontierPolicy, s::StateCen)
    nb_robots = length(s.robots_states)
    robots = [model[i] for i in 1:nb_robots]

    frontiers = abmproperties(model).rollout_parameters.frontiers
    goal = abmproperties(model).rollout_parameters.goal

    if isempty(frontiers)
        out = rand(rollout_policy.mdp.possible_actions)
        return out
    end

    if (count(x->x==(0,0), goal) == 0 && all([s.gridmap[goal[i][1],goal[i][2]]!=-2 for i in 1:nb_robots])) || isempty(abmproperties(model).rollout_parameters.frontiers)
        abmproperties(model).rollout_parameters.frontiers = frontierDetectionMCTS(s.gridmap, abmproperties(model).rollout_parameters.frontiers, need_repartition=false)
    end        

    out = rand(rollout_policy.mdp.possible_actions)
    for robot in robots
        if abmproperties(model).rollout_parameters.goal[robot.id] == (0,0) || s.gridmap[goal[robot.id][1],goal[robot.id][2]]!=-2
            abmproperties(model).rollout_parameters.goal[robot.id] = rand(abmproperties(model).rollout_parameters.frontiers)
        end

        pos = s.robots_states[robot.id].pos
        plan = collect(Agents.Pathfinding.find_path(robot.pathfinder, pos, abmproperties(model).rollout_parameters.goal[robot.id]))

        if isempty(plan)
            abmproperties(model).rollout_parameters.goal[robot.id] = (0,0)
        else
            a = (plan[1][1]-pos[1], plan[1][2]-pos[2])./distance(plan[1], pos)
            best_direction = all_directions[1]
            dist=10000000
            for d in all_directions
                tmp_dist = distance(a,d.direction)
                if tmp_dist < dist
                    dist = tmp_dist
                    best_direction = d
                end
            end
            out.directions[robot.id] = best_direction
        end  
    end
    return out
end


function special_Q(m::RobotMDP, s::StateCen, a::ActionCen)
    next_pos = [rs.pos for rs in s.robots_states]
    for rs in s.robots_states
        next_pos[rs.id], _ = compute_new_pos(s.gridmap, rs.id, next_pos, 1, a.directions[rs.id].direction)
    end
    if any(next_pos .== [rs.pos for rs in s.robots_states])
        return -1000000.0
    else
        return 0.0
    end
end



# function special_N(m::RobotMDP, s::StateCen, a::ActionCen)
#     next_pos = [rs.pos for rs in s.robots_states]
#     for rs in s.robots_states
#         next_pos[rs.id], _ = compute_new_pos(s.gridmap, rs.id, next_pos, 1, a.directions[rs.id].direction)
#     end
#     if any(next_pos .== [rs.pos for rs in s.robots_states])
#         return 1000000
#     else
#         return 0
#     end
# end