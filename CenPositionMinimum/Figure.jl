function initialise_figure(observ_map, observ_pos_list, observ_traj_list)
    # nb_robots = length(observ_map)
    # f = Figure()
    # agent_polygon = Makie.Polygon(Point2f[(-0.5, -0.5), (1, 0), (-0.5, 0.5)])
    
    # extent = size(observ_map[1][])
    
    # a = round(Int64, sqrt(nb_robots))
    # b = ceil(Int64, nb_robots/a)

    # compteur = 1

    # for i in 1:a
    #     for j in 1:b
    #         if compteur <= nb_robots
    #             ax = Axis(f[i,j], title = "Robot $(compteur)")
                                
    #             heatmap!(ax, observ_map[compteur], colorrange = (-2,0))
                
    #             for k in 1:nb_robots
    #                 if k == compteur
    #                     scatter!(ax, observ_pos_list[compteur][k], marker = agent_polygon, color = :red)
    #                     lines!(ax, observ_traj_list[compteur], linewidth = 2, color = :black)
    #                 end
    #             end

    #         end
    #         compteur+=1
    #     end
    # end

    # Colorbar(f[end,end+1], colorrange = (-2,0))

    # display(f)
    # return f

    f = Figure()
    ax = Axis(f[1,:])
    
    heatmap!(ax, observ_map, colorrange = (-2,0))
    
    agent_polygon = Makie.Polygon(Point2f[(-0.5, -0.5), (1, 0), (-0.5, 0.5)])
    for i in eachindex(observ_pos_list)
        scatter!(ax, observ_pos_list[i], marker = agent_polygon)
        lines!(ax, observ_traj_list[i], linewidth = 5)
    end

    Colorbar(f[1,end+1], colorrange = (-2,0))

    display(f)
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