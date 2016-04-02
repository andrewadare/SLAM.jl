
include("ekfslam-sim.jl")
include("gr-draw.jl")


# Global object containing simulation state, updated in real time.
# It is exposed for monitoring, debugging, and flexible visualization.
simdata = ekfsim_setup(10, "course1.txt")

init_plot_window(simdata.scene.boundaries)
draw_map(simdata.scene.landmarks, simdata.scene.waypoints)

let ellipses = []

    global monitor

    function monitor(simdata::SimData)
        scene, vehicle, state = simdata.scene, simdata.vehicle, simdata.state
        draw_scene(scene, state, vehicle)
        if simdata.state_updated
            draw_laser_lines(simdata.z, state.x[1:3])
            ellipses = compute_landmark_ellipses(state.x, state.cov)
        end
        draw_landmark_ellipses(ellipses)
        GR.updatews()
        return
    end

end

sim!(simdata, monitor, [simdata])
