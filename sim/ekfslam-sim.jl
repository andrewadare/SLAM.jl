# SLAM simulation using extended Kalman filter. Inspired by Matlab implementation
# by T. Bailey et al (https://openslam.org/bailey-slam.html).

slam_dir = "$(pwd())/.."
slam_dir in LOAD_PATH || push!(LOAD_PATH, slam_dir)

using SLAM

include("sim-utils.jl")
include("gr-draw.jl")

# Scene boundaries: xmin, xmax, ymin, ymax
const BOUNDARIES = [0.; 100; 0; 100]
const N_LANDMARKS = 10::Int

function get_waypoints()
    wp, _ = readdlm("course1.txt", header=true)
    wp = wp'
end

scene = Scene(BOUNDARIES,
              get_waypoints(),
              make_landmarks(N_LANDMARKS, BOUNDARIES, 0.05),
              Array{Float64}(3, 10000),
              Array{Float64}(3, 10000),
              0)

function main()
    
    wp = scene.waypoints               # [x,y] guidance waypoints
    lmtags = 1:N_LANDMARKS             # Unique identifier for each landmark

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
    da_table = zeros(1, N_LANDMARKS)   # Data association table 
    
    # GR.beginprint("racecourse.png") # Bug: outputs gks.png instead
    init_plot_window(BOUNDARIES)
    # GR.endprint()

    draw_map(scene.landmarks, scene.waypoints)

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
            z, tags = get_observations(vehicle.pose, scene.landmarks, lmtags, sensor_range, R)

            zf, idf, zn = associate(state, z, RE, gate_rej, gate_add)

            # Update SLAM state
            state.x, state.cov = update(state, zf, RE, idf)

            # Add z measurements to the SLAM state vector and its covariance
            state.x, state.cov = add_features(state, zn, RE)
        end

        # Visualize
        scene.nsteps += 1
        scene.true_track[:, scene.nsteps] = vehicle.pose
        scene.slam_track[:, scene.nsteps] = state.x[1:3]

        draw_map(scene.landmarks, wp)
        draw_vehicle(frame_transform(vehicle.shape, vehicle.pose))

        if scene.nsteps > 1        
            draw_true_path(scene.true_track, scene.nsteps)
            draw_slam_path(scene.slam_track, scene.nsteps)
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
