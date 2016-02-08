function make_landmarks(nlandmarks::Int, boundaries, margin)
    xmin, xmax, ymin, ymax = boundaries
    bx = margin*(xmax - xmin)
    by = margin*(ymax - ymin)
    rand([xmin + bx:xmax - bx; ymin + by:ymax - by], 2, nlandmarks)
end


function nearby_landmark_indices(pose, lm, rmax)
    # Return indices of landmarks within sensor acceptance
    dx = lm[1,:] - pose[1]
    dy = lm[2,:] - pose[2]
    phi = pose[3]

    inearby = Array{Int}(0)
    for i in 1:size(lm, 2)
        # Select landmarks within +/- 90 degrees of current heading
        if (dx[i]*cos(phi) + dy[i]*sin(phi)) > 0
            # Select landmarks inside sensor range
            if ((dx[i]^2 + dy[i]^2) < rmax^2)
                push!(inearby, i)
            end
        end
    end
    inearby
end

"""
Add Gaussian noise to nominal control values
Noise is taken to be uncorrelated (assume diagonal control_cov)
"""
function add_control_noise!(vehicle::Vehicle, control_cov::AbstractMatrix)

    vehicle.measured_speed = vehicle.target_speed + randn(1)[1]*sqrt(control_cov[1,1])
    vehicle.measured_gamma = vehicle.target_gamma + randn(1)[1]*sqrt(control_cov[2,2])
end


function add_control_noise(speed, gamma, control_cov)
    # Add Gaussian noise to nominal control values
    # Noise is taken to be uncorrelated (assume diagonal control_cov)
    speed + randn(1)[1]*sqrt(control_cov[1,1]),
    gamma + randn(1)[1]*sqrt(control_cov[2,2])
end


function add_observation_noise(z, R)
    # Add Gaussian noise to observation vector z
    ncols = size(z, 2)
    [z[1,:] + randn(1, ncols)*sqrt(R[1,1])
     z[2,:] + randn(1, ncols)*sqrt(R[2,2])]
end

"""
Return array with columns z = [range; angle] for all nearby landmark
observations as well as the corresponding list of landmark IDs.
"""
function get_observations(vehicle::Vehicle, scene::Scene, R)
    lm = scene.landmarks
    x,y,phi = vehicle.pose

    inearby = nearby_landmark_indices(vehicle.pose, lm, vehicle.sensor_range)

    # Unique identifier for each landmark
    landmark_tags = 1:size(lm, 2)

    dx  = lm[1,inearby] - x
    dy  = lm[2,inearby] - y

    z = add_observation_noise([sqrt(dx.^2 + dy.^2); atan2(dy, dx) - phi], R)
    z, landmark_tags[inearby]
end
