import GR


function draw_scene(scene::Scene, state::EKFSlamState, vehicle::Vehicle)
    draw_map(scene.landmarks, scene.waypoints)
    draw_vehicle(frame_transform(vehicle.shape, vehicle.pose))
    if scene.nsteps > 1
        draw_true_path(scene.true_track, scene.nsteps)
        draw_slam_path(scene.slam_track, scene.nsteps)
        draw_vehicle_ellipse(state.x, state.cov)
    end
    draw_slam_landmarks(state.x)
    return
end


function init_plot_window(boundaries)
    xmin, xmax, ymin, ymax = boundaries
    GR.clearws()
    GR.setwsviewport(0.0, 0.25, 0.0, 0.25) # Display window extents in meters
    GR.setviewport(0.15, 0.95, 0.15, 0.95)
    GR.setwindow(xmin, xmax, ymin, ymax)
end


function draw_map(lm, wp)
    draw_axes()
    draw_landmarks(lm)
    draw_waypoints(wp)
end


function draw_axes()
    GR.clearws()
    GR.settransparency(1.0)
    GR.setlinecolorind(1)
    GR.setlinewidth(1)
    GR.axes(10, 10, 0, 0, 2, 2, -0.01)
end


function draw_landmarks(lm)
    GR.settransparency(1.0)
    GR.setmarkersize(1.25)
    GR.setmarkercolorind(4)
    GR.setmarkertype(GR.MARKERTYPE_SQUARE)

    lmx, lmy = vec(lm[1,:]), vec(lm[2,:])
    GR.polymarker(lmx, lmy)
end


function draw_waypoints(wp)
    GR.settransparency(1.0)
    GR.setmarkersize(0.75)
    GR.setmarkercolorind(1)
    GR.setmarkertype(GR.MARKERTYPE_CIRCLE)

    wpx, wpy = vec(wp[1,:]), vec(wp[2,:])
    GR.polymarker(wpx, wpy)
end


"""
Draw true vehicle position
"""
function draw_true_path(x, n)
    GR.settransparency(1.0)
    GR.setlinewidth(2)
    GR.setlinecolorind(981)
    GR.polyline(vec(x[1,1:n]), vec(x[2,1:n]))
end

"""
Draw filled polygon with x,y points given by shape array
"""
function draw_vehicle(shape)

    GR.settransparency(0.8)
    GR.setfillintstyle(1) # solid
    GR.setfillcolorind(981)
    GR.fillarea(vec(shape[1,:]), vec(shape[2,:]))
end


"""
Draw inferred vehicle position
"""
function draw_slam_path(x, n)
    GR.settransparency(1.0)
    GR.setlinewidth(2)
    GR.setlinecolorind(983)
    GR.polyline(vec(x[1,1:n]), vec(x[2,1:n]))
end


"""
Draw observed landmarks (appended to state vector)
"""
function draw_slam_landmarks(x)
    if length(x) < 5
        return
    end
    GR.settransparency(0.8)
    GR.setmarkersize(1.5)
    GR.setmarkercolorind(8)
    GR.setmarkertype(GR.MARKERTYPE_STAR)
    GR.polymarker(vec(x[4:2:end]), vec(x[5:2:end]))
end


function draw_vehicle_ellipse(x, P)

    nsigma = 2 # Size of ellipse
    nsegs = 16 # Number of line segments to use

    ell = ellipse(x[1:2], P[1:2, 1:2], nsigma, nsegs)

    GR.settransparency(0.8)
    GR.setlinewidth(2)
    GR.setlinecolorind(983)
    GR.polyline(vec(ell[1,:]), vec(ell[2,:]))
end



function draw_landmark_ellipses(ellipses)

    n = floor(Int, size(ellipses,1)/2)
    if n == 0
        return
    end

    GR.settransparency(0.5)
    GR.setlinecolorind(1)

    for i = 1:n
        GR.polyline(vec(ellipses[2i-1,:]), vec(ellipses[2i,:]))
    end
end


function draw_laser_lines(z, pose)
    GR.settransparency(1.0)
    GR.setlinecolorind(2) # red
    GR.setlinewidth(2)
    lines = laser_lines(z, pose)
    for i = 1:size(lines, 2)
        GR.polyline([lines[1,i]; lines[3,i]], [lines[2,i]; lines[4,i]])
    end
end

