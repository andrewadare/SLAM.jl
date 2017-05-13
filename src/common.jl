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
    pose::Vector{T}                    # Inferred vehicle pose e.g. [x, y, phi]
    pcov::Matrix{T}                    # Pose covariance matrix
    features::Matrix{T}                # Feature positions in columns
    fcovs::Array{T}                    # Feature/landmark covariance matrices
    weight::T                          # Weight factor for this particle
end

abstract SlamState

# Extended Kalman Filter SLAM state and covariance
type EKFSlamState{T<:Real} <: SlamState
    x::Vector{T}                       # Joint pose + feature state
    cov::Matrix{T}                     # Joint pose + feature cov matrix
end

# Particle Filter SLAM state
type PFSlamState{T<:Real} <: SlamState
    x::Vector{T}
    cov::Matrix{T}
    nparticles::Int
    particles::Vector{Particle{T}}
end

# PFSlamState Constructor
function PFSlamState{T}(nparticles::Int, initial_pose::Vector{T})
    particles = Vector{Particle{T}}(0)
    pose_ndof = length(initial_pose)
    initial_cov = zeros(T, pose_ndof, pose_ndof)

    for i = 1:nparticles

        # Vehicle pose and covariance
        pose = initial_pose
        pcov = zeros(T, pose_ndof, pose_ndof)

        # Feature/landmark positions and covariances.
        # Final index is for landmarks.
        features = Matrix{T}(2, 0)
        fcovs = Array{T}(2, 2, 0)

        # Initial weighting
        weight = 1.0/nparticles

        p = Particle(pose, pcov, features, fcovs, weight)
        push!(particles, p)
    end

    return PFSlamState(initial_pose, initial_cov, nparticles, particles)
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
    z::Matrix{T}             # Recent [range, bearing] feature observations
    nz::Int                  # Current number of observations
    state_updated::Bool      # If SLAM state occured in this step
    paused::Bool             # Paused or running
    nlaps::Int               # Number of loops through the waypoint list
end


## Functions ##


"""
Reset EKFSlamState to initial configuration
"""
function reset!{T}(state::EKFSlamState{T})
    state.x = zeros(T, 3)
    state.cov = zeros(T, 3, 3)
end


"""
Reset PFSlamState to initial configuration
"""
function reset!{T}(state::PFSlamState{T})
    for particle in state.particles
        reset!(particle)
    end
end


"""
Reset Particle to initial configuration
"""
function reset!{T}(particle::Particle{T})
    # TODO
end


"""
Returns true if (x,y) falls within scene.boundaries
"""
function inbounds(x::Real, y::Real, scene::Scene)
    # Scene boundaries: xmin, xmax, ymin, ymax
    xmin, xmax, ymin, ymax = scene.boundaries
    return xmin <= x <= xmax && ymin <= y <= ymax
end


"""
Read x,y positions from 2-column text file and return as 2 x N array
"""
function get_waypoints(txtfile::AbstractString)
    wp, _ = readdlm(txtfile, header=true)
    return wp'
end


"""
Initial [x, y, phi] at first waypoint, heading for second waypoint
"""
function initial_pose(scene::Scene)
    wp = scene.waypoints
    return [wp[1,1]; wp[2,1]; atan2(wp[2,2] - wp[2,1], wp[1,2] - wp[1,1])]
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
    return phi
end


"""
Calculate the Kalman update given the prior state [x,P], the innovation
[v,R] and the linear observation model H.

The result is calculated using Cholesky factorisation, which is more
numerically stable than a naive implementation.
"""
function kalman_cholesky_update(x, P, v, R, H)
    PHt = P*H'
    S = H*PHt + R
    S = (S + S')*0.5 # make symmetric
    C = inv(chol(S)) # triangular matrix
    W1 = PHt*C
    W = W1*C'

    x_new = x + W*v
    P_new = P - W1*W1'

    return x_new[:], P_new  # Make x_new size (3,) vs. (3,1)
end


"""
Parameters:
x: mean vector
P: covariance matrix
n: number of samples to collect
"""
function rand_mvn{T}(x::Vector{T}, P::Matrix{T}, n::Integer)
    S = chol(Hermitian(P))'
    X = randn(length(x), n)
    return S*X + x*ones(1, n)
end


"""
Evaluate multivariate normal distribution for a set of innovation vectors and
their covariance matrix. Used for computing (log) likelihood values. For
numerical stability, a Cholesky decomposition of the covariance matrix is used
instead of a direct approach.

Parameters:
v: matrix whose columns are innovation vectors
vcov: covariance matrix of v

Returns:
a vector of (log) Gaussian values.
"""
function eval_mvn{T}(v::Matrix{T}, vcov::Matrix{T}; eval_log=false)
    d = size(v, 1)  # Dimensionality of each innovation vector
    chol_cov = chol(vcov)'
    v_normalized = chol_cov\v

    # Compute vector of Gaussian exponential arguments (one entry for each
    # innovation vector)
    arg = -0.5*sum(v_normalized.^2, 1)

    if eval_log
        result = arg - 0.5*d*log(2pi) - sum(log(diag(chol_cov)))
    else
        result = exp(arg)/((2pi)^(d/2)*prod(diag(chol_cov)))
    end
    return result
end


"""
Given a 2-by-n or 3-by-n matrix `l` whose columns are positions (x,y) or
poses (x,y,phi) in the local vehicle coordinate system, and a vehicle pose `g`
in the global coordinate system, return `l` transformed to global coordinates.
"""
function local_to_global(l, g)

    q = zeros(l)
    phi = g[3]

    # Rotate and translate
    R = [cos(phi) -sin(phi); sin(phi) cos(phi)]
    q[1:2,:] = R*l[1:2,:] .+ g[1:2]

    # if l is a pose (includes an azimuthal angle)
    if size(l,1)==3
        q[3,:] = mpi_to_pi(l[3,:] .+ phi)
    end
    return q
end


function jacobians{T, U<:Integer}(particle::Particle{T},
                                  feature_ids::Vector{U},
                                  R::Matrix{T})
    M = length(feature_ids)

    @assert(size(particle.features, 2) >= M,
        "$(particle.features) particle features < $M feature IDs")

    zp = Matrix{T}(2, M)
    Hv = Array{T}(2, 3, M)
    Hf = Array{T}(2, 2, M)
    Sf = Array{T}(2, 2, M)

    for i = 1:M
        dx = particle.features[1, i] - particle.pose[1]
        dy = particle.features[2, i] - particle.pose[2]
        r2 = dx^2 + dy^2
        r = sqrt(r2)

        # Predicted observation
        zp[:,i] = [r; mpi_to_pi(atan2(dy, dx) - particle.pose[3])]

        # Jacobian matrix wrt particle state
        Hv[:,:,i] = [-dx/r -dy/r   0;
                     dy/r2 -dx/r2 -1]

        # Jacobian wrt feature state
        Hf[:,:,i] = [ dx/r   dy/r;
                     -dy/r2 dx/r2]

        # Innovation covariance of feature observation given the vehicle
        Sf[:,:,i] = Hf[:,:,i] * Pf[:,:,i] * Hf[:,:,i]' + R
    end

    return zp, Hv, Hf, Sf
end


"""
Update pose and covariance with an externally supplied heading angle phi (e.g.
from IMU or GPS) and its uncertainty sigma_phi.
"""
function fuse_heading_measurement{T}(x_prior::Vector{T},
                                     cov_prior::Matrix{T},
                                     phi::T,
                                     sigma_phi::T)
    phi_prior = x_prior[3]

    # Measurement model
    H = [0 0 1]

    # Innovation
    v = mpi_to_pi(phi - phi_prior)

    x, P = kalman_cholesky_update(x_prior, cov_prior, v, sigma_phi^2, H)

    return x, P
end


"""
Predict SLAM pose and covariance matrix according to equations of motion for vehicle.
Q is the covariance matrix for speed and steering angle gamma.
dt is the timestep
"""
function predict_pose{T<:Number}(x::Vector{T},
                                 P::Matrix{T},
                                 vehicle::Vehicle,
                                 Q::AbstractMatrix,
                                 dt::AbstractFloat)
    phi = x[3]

    g = vehicle.measured_gamma
    v = vehicle.measured_speed
    w = vehicle.wheelbase

    s = sin(g + phi)
    c = cos(g + phi)
    vts = v*dt*s
    vtc = v*dt*c

    # Jacobians
    Gv = [1 0 -vts;
          0 1 vtc;
          0 0 1]
    Gu = [dt*c -vts;
          dt*s  vtc;
          dt*sin(g)/w v*dt*cos(g)/w]

    # Predict pose
    new_x = [x[1] + vtc;
             x[2] + vts;
             mpi_to_pi(phi + v*dt*sin(g)/w)]

    # Predict covariance
    new_P = Gv*P*Gv' + Gu*Q*Gu'

   return new_x, new_P, Gu, Gv
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

    return z, H
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

    return nsigma*sqrtm(P)*[cos(phi)'; sin(phi)'] .+ x[1:2]
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
    return ellipses
end


"""
Return matrix of line segments for laser range-bearing measurements.
Each column contains vehicle and feature positions: [vx vy fx fy]'
"""
function laser_lines(z, vehicle_pose)
    nlines = size(z, 2)
    lines = Array{Float64}(4, nlines)
    lines[1,:] = zeros(1, nlines) .+ vehicle_pose[1]
    lines[2,:] = zeros(1, nlines) .+ vehicle_pose[2]

    # Range and bearing from vehicle to observations
    r, b = z[1,:], z[2,:]

    # x,y position of observations in local (vehicle) coordinate frame
    xy_local = [r.*cos(b) r.*sin(b)]'

    lines[3:4,:] = local_to_global(xy_local, vehicle_pose)
    return lines
end
