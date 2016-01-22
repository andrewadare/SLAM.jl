import GR

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

function ellipse(x, P, nsigma, nsegs)
    # Make a single 2D n-sigma Gaussian contour centered at x with axes given
    # by covariance matrix P

    # Approximate smooth ellipse with nsegs line segments (nsegs + 1 points)
    phi = 0:2*pi/nsegs:2*pi

    nsigma*sqrtm(P)*[cos(phi)'; sin(phi)'] .+ x[1:2]
end


function compute_landmark_ellipses(x, P)
    nsigma = 2 # Size of ellipse
    nsegs = 12 # Number of line segments to use

    # Number of observed features/landmarks
    nf = floor(Int, (length(x) - 3)/2)

    # Allocate rows for x,y and columns for npoints/ellipse times # ellipses
    ellipses = Array{Float64}(2*nf, nsegs + 1)

    for i = 1:nf
        j = (2*i + 2):(2*i + 3)

        ell = ellipse(x[j], P[j,j], nsigma, nsegs)
        ellipses[2i-1:2i, :] = ellipse(x[j], P[j,j], nsigma, nsegs)
    end
    ellipses
end


function laser_lines(z,x)
    # Return array of line segments for laser range-bearing measurements.
    # Columns contain vehicle and feature positions [vx; vy; fx; fy]
    len = size(z, 2)
    lines = Array{Float64}(4, len)
    lines[1,:] = zeros(1, len) + x[1]
    lines[2,:] = zeros(1, len) + x[2]
    lines[3:4,:] = frame_transform([z[1,:].*cos(z[2,:]); z[1,:].*sin(z[2,:])], x)
    lines
end


