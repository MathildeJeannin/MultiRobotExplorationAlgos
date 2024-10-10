function initialise_figure(observ_map, observ_pos_list, observ_traj_list, com_range, observ_best_sequences_list)
    nb_robots = length(observ_map)
    f = Figure()
    agent_polygon = Makie.Polygon(Point2f[(-0.5, -0.5), (1, 0), (-0.5, 0.5)])
    
    extent = size(observ_map[1][])
    
    a = round(Int64, sqrt(nb_robots))
    b = ceil(Int64, nb_robots/a)

    compteur = 1

    for i in 1:a
        for j in 1:b
            if compteur <= nb_robots
                ax = Axis(f[i,j], title = "Robot $(compteur)")
                                
                heatmap!(ax, observ_map[compteur], colorrange = (-2,0))
                
                # for (i,position) in enumerate(observ_pos_list)
                #     if i!=compteur
                #         scatter!(ax, observ_pos_list[compteur][i], marker = agent_polygon, color = :green)
                #     else
                #         scatter!(ax, observ_pos_list[i], marker = agent_polygon, color = :red)
                #     end
                # end

                for k in 1:nb_robots
                    if k == compteur
                        # lines!(ax, observ_best_sequences_list[compteur][k], width = 5, color = :red)
                        scatter!(ax, observ_pos_list[compteur][k], marker = agent_polygon, color = :red)
                        lines!(ax, observ_traj_list[compteur], linewidth = 2, color = :black)
                    else
                        # lines!(ax, observ_best_sequences_list[compteur][k], width = 5, color = :green)
                        # scatter!(ax, observ_pos_list[compteur][k], marker = agent_polygon, color = :green)
                    end
                end

            end
            compteur+=1
        end
    end

    Colorbar(f[end,end+1], colorrange = (-2,0))

    display(f)
    return f
end


function _circle(pos,r)
    x,y = pos[1],pos[2]
    θ = LinRange(0,2*π,100)
    poses = []
    for el in θ
        push!(poses, Point2f(x + r*sin(el), y+ r*cos(el)))
    end
    return poses
end