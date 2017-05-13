
function sim!(simdata::SimData,
              scene::Scene,
              vehicle::Vehicle,
              state::PFSlamState,
              monitor::Function,
              monitor_args::AbstractVector)

    n_landmarks = size(scene.landmarks, 2)

    # Data association list for the known DA problem (fixed number of landmarks)
    da_list = zeros(1, n_landmarks)

    d_min = 1.0                  # [m] Once inside d_min, head for next waypoint

    # Simulated control noise covariance matrix
    sigma_speed = 0.5            # [m/s] Uncertainty about target speed
    sigma_steer = (3.0*pi/180)   # [rad] Uncertainty about target gamma
    Q = [sigma_speed^2 0;        # Process covariance matrix
         0 sigma_steer^2]

    # Simulated observation noise covariance matrix
    sigmaR = 0.1                 # [m] Range uncertainty
    sigmaB = (1.0*pi/180)        # [rad] Bearing angle uncertainty
    R = [sigmaR^2 0;             # Obs. covariance matrix
         0 sigmaB^2]

    # Timing variables
    dt = 0.025                   # [s] Interval between control updates
    dt_obs = 8*dt                # [s] Interval between state updates
    dtsum = 0                    # Time since last observation
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

        # Predict new state and covariance of each particle from motion model
        for (i, p) in enumerate(state.particles)
            p.pose, p.pcov, _, _ = predict_pose(p.pose, p.pcov, vehicle, Q, dt)

            # Simulate an IMU heading measurement with 2 degree uncertainty
            # and fuse it into the state estimate
            sigma_phi = 2*pi/180
            phi_imu = vehicle.pose[3] + sigma_phi*randn()
            p.pose, p.pcov = fuse_heading_measurement(p.pose, p.pcov, phi_imu, sigma_phi)
        end

        dtsum += dt

        # Update step - takes place every (dt_obs/dt) timesteps
        if dtsum > dt_obs
            dtsum = 0
            simdata.state_updated = true
            simdata.z, tags = get_observations(vehicle, scene, R)
            simdata.nz = size(simdata.z, 2)

            # Number of features observed so far
            nf = size(state.particles[1].features, 2)

            # Compute known data association
            zf, idf, zn = associate_known!(simdata.z, tags, da_list, nf)

            # Update estimate of existing features
            if size(zf, 2) > 0
                for (i, p) in enumerate(state.particles)
                    println("features: ", p.features, ", size(zf) = ", size(zf), ", nf = $nf")
                    p.pose, p.pcov, p.weight = sample_proposal(p, zf, tags, R)

                    # TODO: feature_update
                end

                # TODO: resample_particles
            end

            # Handle new features
            if size(zn, 2) > 0
                for (i, p) in enumerate(state.particles)
                    # Sample from proposal if not already done above
                    if size(zn, 2) > 0
                        p.pose = vec(rand_mvn(p.pose, p.pcov, 1))
                        p.pcov = zeros(3, 3)
                    end
                    add_features!(p, zn, R)
                end
            end

        else
            simdata.state_updated = false
        end

        # Update vehicle tracks in scene
        scene.nsteps += 1
        scene.true_track[:, scene.nsteps] = vehicle.pose
        scene.slam_track[:, scene.nsteps] = vehicle.pose #state.x[1:3]

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
