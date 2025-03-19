function initialise_figure(observ_map, observ_pos_list, observ_traj_list)
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
