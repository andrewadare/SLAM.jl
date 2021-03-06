# SLAM simulation using extended Kalman filter. Inspired by Matlab implementation
# by T. Bailey et al (https://openslam.org/bailey-slam.html).

# Here are three options to make the SLAM module (or any other) available to your code:
#
# 1. Include it directly: `include("path/to/SLAM.jl/src/SLAM.jl")`
# 2. Add it to Julia's LOAD_PATH environment variable: if `module_dir = "/path/to/SLAM.jl"`,
#    add a line like `module_dir in LOAD_PATH || push!(LOAD_PATH, module_dir)` to your code,
#    your REPL session, or to `~/.juliarc.jl`.
# 3. Use SLAM.jl as a package: Pkg.clone("url/to/SLAM.jl")
include("../src/SLAM.jl")

using SLAM

include("sim-utils.jl")

function ekfsim_setup(n_landmarks::Integer, waypoints_file::AbstractString)

    # Scene boundaries: xmin, xmax, ymin, ymax
    boundaries = [0.; 100; 0; 100]
    scene = Scene(boundaries,
                  get_waypoints(waypoints_file),
                  make_landmarks(n_landmarks, boundaries, 0.05),
                  Array{Float64}(3, 10000),
                  Array{Float64}(3, 10000),
                  0)

    # Vehicle
    vehicle = Vehicle()
    vehicle.wheelbase = 4.0            # [m]
    vehicle.max_gamma = 60*pi/180      # [rad] max steering angle (-max < g < max)
    vehicle.steer_rate = 60*pi/180     # [rad/s] max rate of change in steer angle
    vehicle.sensor_range = 30          # [m] Landmark detection radius
    vehicle.shape = [1. -1 -1; 0 1 -1] # A little triangle for visualization
    vehicle.pose = initial_pose(scene)
    vehicle.target_speed = 8           # [m/s]
    vehicle.waypoint_id = 1            # Initialize to index of first waypoint

    # SLAM state vector
    # First three elements comprise the SLAM vehicle pose; state is augmented as
    # new landmarks are observed. Covariance matrix is also augmented during simulation.
    state = EKFSlamState(vehicle.pose, zeros(3, 3))

    SimData(scene, vehicle, state, zeros(2, n_landmarks), 0, false, false, 2)
end


# This method runs the simulation without monitoring
function sim!(simdata::SimData)
    sim!(simdata::SimData, (x...) -> (), [])
end


function sim!(simdata::SimData, monitor::Function, args::AbstractVector)

    # Use "typedefs" for simdata fields to shorten names.
    # Since these are references, the simdata object is modified in place.
    scene, vehicle, state = simdata.scene, simdata.vehicle, simdata.state

    n_landmarks = size(scene.landmarks, 2)

    d_min = 1.0                        # [m] Once inside d_min, head for next waypoint

    # Simulated control noise covariance matrix
    sigma_speed = 0.5                  # [m/s] Uncertainty about target speed
    sigma_steer = (3.0*pi/180)         # [rad] Uncertainty about target gamma
    Q = [sigma_speed^2 0; 0 sigma_steer^2]

    # Simulated observation noise covariance matrix
    sigmaR = 0.1                       # [m] Range uncertainty
    sigmaB = (1.0*pi/180)              # [rad] Bearing angle uncertainty
    R = [sigmaR^2 0; 0 sigmaB^2]       # Obs. covariance matrix

    # Timing variables
    dt = 0.025                         # [s] Interval between control updates
    dt_obs = 8*dt                      # [s] Interval between state updates
    dtsum = 0                          # Time since last observation
    marker = time()

    while vehicle.waypoint_id != 0

        start = marker

        # Update heading and target waypoint
        steer!(vehicle, scene.waypoints, d_min, dt)

        # If lap through waypoints is finished with more to go, start a new lap
        if vehicle.waypoint_id == 0 && simdata.nlaps > 1
            vehicle.waypoint_id = 1
            simdata.nlaps -= 1
        end

        # Advance vehicle's equations of motion by one timestep
        step_vehicle!(vehicle, dt)

        # Simulate imprecision about control target speed and steering angle
        add_control_noise!(vehicle, Q)

        # Prediction update for state vector and covariance
        state.x, state.cov = predict(state, vehicle, Q, dt)

        dtsum += dt

        # EKF update step - takes place every (dt_obs/dt) timesteps
        if dtsum > dt_obs
            dtsum = 0
            simdata.state_updated = true
            simdata.z, tags = get_observations(vehicle, scene, R)
            simdata.nz = size(simdata.z, 2)

            # Last two parameters are thresholds (Mahalanobis distances)
            # 4.0  [m] max distance for association
            # 25.0 [m] min distance for creation of new feature
            zf, idf, zn = associate(state, simdata.z, R, 4.0, 25.0)

            # Update SLAM state
            state.x, state.cov = update(state, zf, R, idf)

            # Add z measurements to the SLAM state vector and its covariance
            state.x, state.cov = add_features(state, zn, R)
        else
            simdata.state_updated = false
        end

        # Update vehicle tracks in scene
        scene.nsteps += 1
        scene.true_track[:, scene.nsteps] = vehicle.pose
        scene.slam_track[:, scene.nsteps] = state.x[1:3]

        monitor(args...)

        # Limit frame rate for realistic timing
        marker = time()
        if start + dt > marker
            sleep(start + dt - marker)
        end

        while simdata.paused
            wait()
        end

    end
end
