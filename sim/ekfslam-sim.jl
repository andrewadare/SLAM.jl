# SLAM simulation using extended Kalman filter. This is essentially a Julia port
# of Matlab code by T. Bailey et al (https://openslam.org/bailey-slam.html).

slam_dir = "$(pwd())/.."
slam_dir in LOAD_PATH || push!(LOAD_PATH, slam_dir)

using SLAM

include("sim-utils.jl")
include("gr-draw.jl")


function main()

    ## Scene boundaries: xmin, xmax, ymin, ymax ##
    boundaries = [0; 100; 0; 100]

    ## Landmarks ##
    nlandmarks = 10
    lm = make_landmarks(nlandmarks, boundaries, 0.05)
    lmtags = 1:size(lm, 2)             # Unique identifier for each landmark

    ## Waypoints ##
    wp, _ = readdlm("course1.txt", header=true)
    wp = wp'
    d_min = 1.0                        # [m] Once inside d_min, head for next waypoint
    nlaps = 2                          # Number of loops through the waypoint list

    # Initial [x, y, phi] at first waypoint, heading for second waypoint
    initial_pose = [wp[1,1];
                    wp[2,1];
                    atan2(wp[2,2] - wp[2,1], wp[1,2] - wp[1,1])]

    ## SLAM state vector ## 
    # First three elements comprise the SLAM vehicle pose; state is augmented as 
    # new landmarks are observed. Covariance matrix is also augmented during simulation.
    state = EKFSlamState(initial_pose, zeros(3,3))

    ## Vehicle ##
    vehicle = Vehicle()
    vehicle.wheelbase = 4.0            # [m]
    vehicle.max_gamma = 60*pi/180      # [rad] max steering angle (-max < g < max)
    vehicle.steer_rate = 60*pi/180     # [rad/s] max rate of change in steer angle
    vehicle.pose = initial_pose
    vehicle.target_speed = 8           # [m/s]
    vehicle.waypoint_id = 1            # Initialize to index of first waypoint
    vehicle.shape = [1. -1 -1; 0 1 -1] # A little triangle for visualization

    ## Control parameters and uncertainties ##
    dt = 0.025                         # [s] Interval between control updates
    sigma_speed = 0.5                  # [m/s] Uncertainty about target speed
    sigma_steer = (3.0*pi/180)         # [rad] Uncertainty about target gamma
    Q = [sigma_speed^2 0; 0 sigma_steer^2]
    QE = Q

    ## Observation parameters ##
    sensor_range = 30                  # [m] Landmark detection radius
    dt_obs = 8*dt                      # [s] Observation timestep
    sigmaR = 0.1                       # [m] Range uncertainty
    sigmaB = (1.0*pi/180)              # [rad] Bearing angle uncertainty
    R = [sigmaR^2 0; 0 sigmaB^2]       # Obs. covariance matrix
    RE = R

    ## Data association parameters ##
    # Innovation thresholds or "gates" (Mahalanobis distances)
    gate_rej = 4.0                     # [m] max distance for association
    gate_add = 25.0                    # [m] min distance for creation of new feature

    ## Bookkeeping variables ##
    dtsum = 0                          # Time since last observation
    da_table = zeros(1, size(lm, 2))   # Data association table 
    nsteps = 0                         # Timestep counter

    # Ideal and SLAM pose histories for visualization.
    # Preallocation estimate may need revision if parameters change
    xtrue_history = Array{Float64}(3, 10000)
    xslam_history = Array{Float64}(3, 10000)
    
    # GR.beginprint("racecourse.png") # Bug: outputs gks.png instead
    init_plot_window(boundaries)
    # GR.endprint()

    draw_map(lm, wp)

    # Uncomment to wait for user input (any key)    
    # readline(STDIN)

    ellipses = []

    while vehicle.waypoint_id != 0

        # Update heading and target waypoint
        steer!(vehicle, wp, d_min, dt)

        # If lap through waypoints is finished with more to go, start a new lap
        if vehicle.waypoint_id == 0 && nlaps > 1
            vehicle.waypoint_id = 1 
            nlaps -= 1 
        end

        # Advance vehicle's equations of motion by one timestep
        step_vehicle!(vehicle, dt)

        # Simulate imprecision about control target speed and steering angle
        add_control_noise!(vehicle, Q)
       
        # Prediction update for state vector and covariance
        state.x, state.cov = predict(state, vehicle, QE, dt)

        dtsum += dt

        # EKF update step
        if dtsum > dt_obs
            dtsum = 0
            z, tags = get_observations(vehicle.pose, lm, lmtags, sensor_range, R)

            zf, idf, zn = associate(state, z, RE, gate_rej, gate_add)

            # Update SLAM state
            batch_update!(state, zf, RE, idf)

            # Add z measurements to the SLAM state vector and its covariance
            augment!(state, zn, RE)
        end

        # Visualize
        nsteps += 1
        xtrue_history[:, nsteps] = vehicle.pose
        xslam_history[:, nsteps] = state.x[1:3]

        draw_map(lm, wp)
        draw_vehicle(frame_transform(vehicle.shape, vehicle.pose))

        if nsteps > 1        
            draw_true_path(xtrue_history, nsteps)
            draw_slam_path(xslam_history, nsteps)
            draw_vehicle_ellipse(state.x, state.cov)
        end

        draw_slam_landmarks(state.x)

        if dtsum == 0
            draw_laser_lines(z, state.x[1:3])
            ellipses = compute_landmark_ellipses(state.x, state.cov)
        end
        draw_landmark_ellipses(ellipses)

        GR.updatews()

    end
end

main()
