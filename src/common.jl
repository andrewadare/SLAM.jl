## common.jl: Core types and shared functions ##

## Core Types ##

type Scene{T<:Real}
    boundaries::Vector{T}              # Scene boundaries
    waypoints::Matrix{T}               # Waypoint positions
    landmarks::Matrix{T}               # True (simulated) feature positions
    true_track::Array{T}               # Ideal pose history
    slam_track::Array{T}               # SLAM pose history
    nsteps::Integer                    # Timestep counter
end

type Particle{T<:Real}
    pose::Pose2D{T}                    # Inferred vehicle pose (x, y, phi)
    pcov::Matrix{T}                    # Pose covariance matrix (3 x 3)

    features::Matrix{T}                # Feature positions (as columns)
    fcov::Vector{Matrix{T}}            # Feature cov matrices
end

type Pose2D{T:<Real}
    x::T                               # x coordinate
    y::T                               # y coordinate
    phi::T                             # Azimuthal heading
end

abstract SlamState

# Extended Kalman Filter SLAM state and covariance
type EKFSlamState{T<:Real} <: SlamState
    x::Vector{T}                       # Joint pose + feature state
    cov::Matrix{T}                     # Joint pose + feature cov matrix
end

# Particle Filter SLAM state and covariance
type PFSlamState{T<:Real} <: SlamState
    particles::Vector{Particle{T}}     # Ensemble of particles
    da_table::Vector{T}                # Data association table
end

type Vehicle{T<:Real}

    ## Intrinsic properties ##
    wheelbase::T     # Front-to-rear wheel separation [m]
    max_gamma::T     # Max steering angle [rad] (-max_g < g < max_g)
    steer_rate::T    # Maximum rate of change in steer angle [rad/s]
    sensor_range::T  # [m] Landmark detection radius
    shape::Matrix{T} # Set of x,y points defining a polygon for visualization

    ## (Potentially) time-varying properties ##
    pose::Vector{T}  # [x; y; phi]

    target_speed::T  # Velocity magnitude (control setpoint) [m/s]
    measured_speed::T  # Measured velocity magnitude [m/s]

    target_gamma::T  # Target steering angle [rad]
    measured_gamma::T  # Measured steering angle [rad]

    waypoint_id::Int  # Index of current target waypoint
end

Vehicle() = Vehicle(0., 0., 0., 0., zeros(2,0), zeros(3), 0., 0., 0., 0., 0)

type SimData{T<:Real}
    scene::Scene{T}
    vehicle::Vehicle{T}
    state::SlamState
    z::Matrix{T}             # Recent [range, bearing] feature observations
    nz::Int                  # Current number of observations
    state_updated::Bool      # If SLAM state occured in this step
    paused::Bool             # Paused or running
    nlaps::Int               # Number of loops through the waypoint list
end

## Functions ##

"""
Read x,y positions from 2-column text file and return as 2 x N array
"""
function get_waypoints(txtfile::AbstractString)
    wp, _ = readdlm(txtfile, header=true)
    wp = wp'
end


"""
Initial [x, y, phi] at first waypoint, heading for second waypoint
"""
function initial_pose(scene::Scene)
    wp = scene.waypoints
    [wp[1,1];
     wp[2,1];
     atan2(wp[2,2] - wp[2,1], wp[1,2] - wp[1,1])]
end


"""
Wrap phi into -pi <= phi <= pi
"""
function mpi_to_pi(phi::AbstractFloat)
    if phi > pi
        return phi - 2*pi
    end
    if phi < -pi
        return phi + 2*pi
    end
    phi
end


"""
Return a rotated and translated pose given the pose l in a local frame and the
pose g in the global frame. l and g should be vectors whose first three elements
are x, y, and phi.
"""
function frame_transform(l, g)

    q = zeros(l)

    # Rotate and translate
    R = [cos(g[3]) -sin(g[3]); sin(g[3]) cos(g[3])]
    q[1:2,:] = R*l[1:2,:] .+ g[1:2]

    # if l is a pose and not a point
    if size(l,1)==3
        q[3,:] = mpi_to_pi(l[3,:] + g[3])
    end
    q
end


"""
Predict the expected range-bearing observation z (and Jacobian H) of
feature/landmark idf in joint pose + landmark state vector x
"""
function predict_observation(x::AbstractVector, idf::Int)

    # Map feature index idf to position fpos in x
    pose_length = 3
    fpos = pose_length + idf*2 - 1

    # Distance from vehicle to landmark
    dx = x[fpos] - x[1]
    dy = x[fpos + 1] - x[2]
    d2 = dx^2 + dy^2
    d = sqrt(d2)

    # Predict [range, bearing] observation z
    z = [d; atan2(dy,dx) - x[3]]

    # Calculate Jacobian H for pose and z
    xd = dx/d
    yd = dy/d
    xd2 = dx/d2
    yd2 = dy/d2

    H = zeros(2, length(x))
    H[:,1:3]         = [-xd -yd 0; yd2 -xd2 -1]
    H[:,fpos:fpos+1] = [ xd  yd;  -yd2  xd2]

    z, H
end


"""
Advance equations of motion for vehicle by dt and update vehicle pose in-place
This uses the ideal/target speed and steering angle
"""
function step_vehicle!(vehicle::Vehicle, dt::AbstractFloat)

    x, y, phi = vehicle.pose
    v = vehicle.target_speed
    g = vehicle.target_gamma

    vehicle.pose = [x + v*dt*cos(g + phi);
                    y + v*dt*sin(g + phi);
                    mpi_to_pi(phi + v*dt*sin(g)/vehicle.wheelbase)]
end


"""
Find the next target waypoint given the current position and steer for it.
d_min is minimum distance to current waypoint before switching to next
This uses the ideal/target steering angle
"""
function steer!(vehicle::Vehicle, waypoints::AbstractArray, d_min::AbstractFloat, dt::AbstractFloat)

    # Get current vehicle steering angle and waypoint index
    g = vehicle.target_gamma
    iwp = vehicle.waypoint_id
    x, y, phi = vehicle.pose

    # Get coords of current waypoint
    cwp = waypoints[:, iwp]

    # Determine if current waypoint has been reached
    d2 = (cwp[1] - x)^2 + (cwp[2] - y)^2
    if d2 < d_min^2
        iwp += 1             # Advance to next waypoint
        if iwp > size(waypoints, 2) # Final waypoint reached, flag and return
            iwp = 0
            vehicle.waypoint_id = iwp
            return
        end
        cwp = waypoints[:, iwp] # Coords of next waypoint
    end

    # Change in steering angle required to point towards current waypoint
    dg = mpi_to_pi(atan2(cwp[2] - y, cwp[1] - x) - phi - g)

    # Constrain steering angle by max steering rate
    dgmax = vehicle.steer_rate*dt
    if abs(dg) > dgmax
        dg = sign(dg)*dgmax
    end

    # Limit steering angle to max steering angle
    g += dg
    if abs(g) > vehicle.max_gamma
        g = sign(g)*vehicle.max_gamma
    end

    vehicle.target_gamma = g
    vehicle.waypoint_id = iwp

    return
end

"""
Make a single 2D n-sigma Gaussian contour centered at x with axes given
by covariance matrix P
"""
function ellipse(x, P, nsigma, nsegs)

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

        ellipses[2i-1:2i, :] = ellipse(x[j], P[j,j], nsigma, nsegs)
    end
    ellipses
end

"""
Return array of line segments for laser range-bearing measurements.
Columns contain vehicle and feature positions [vx; vy; fx; fy]
"""
function laser_lines(z, x)
    len = size(z, 2)
    lines = Array{Float64}(4, len)
    lines[1,:] = zeros(1, len) + x[1]
    lines[2,:] = zeros(1, len) + x[2]
    lines[3:4,:] = frame_transform([z[1,:].*cos(z[2,:]); z[1,:].*sin(z[2,:])], x)
    lines
end
