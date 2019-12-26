
function sim!(simdata::SimData,
              scene::Scene,
              vehicle::Vehicle,
              state::EKFSlamState,
              monitor::Function,
              monitor_args::AbstractVector)

    n_landmarks = size(scene.landmarks, 2)

    d_min = 1.0                        # [m] Once inside d_min, head for next waypoint

    # Simulated control noise covariance matrix
    sigma_speed = 0.5                  # [m/s] Uncertainty about target speed
    sigma_steer = (3.0 * pi / 180)         # [rad] Uncertainty about target gamma
    Q = [sigma_speed^2 0; 0 sigma_steer^2]

    # Simulated observation noise covariance matrix
    sigmaR = 0.1                       # [m] Range uncertainty
    sigmaB = (1.0 * pi / 180)              # [rad] Bearing angle uncertainty
    R = [sigmaR^2 0; 0 sigmaB^2]       # Obs. covariance matrix

    # Timing variables
    dt = 0.025                         # [s] Interval between control updates
    dt_obs = 8 * dt                      # [s] Interval between state updates
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
        state.x, state.cov = predict_pose(state, vehicle, Q, dt)

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

        monitor(monitor_args...)

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
