
"""
Parameters:
particle: instance of a Particle
z: Observation array with columns = [range; angle]
tags: Identity of landmarks
R: 2x2 observation covariance matrix

Returns:
x: proposal mean (vector)
P: proposal covariance (matrix)
w: proposal weight (scalar)
"""
function sample_proposal{T, U<:Integer}(particle::Particle{T},
                                        z::Matrix{T},
                                        feature_ids::Vector{U},
                                        R::Matrix{T})
    x = copy(particle.pose)
    P = copy(particle.pcov)

    for i = 1:length(feature_ids)
        j = feature_ids[i]
        zpi, Hvi, Hfi, Sf = jacobian(particle, j, R)
        Sfi = inv(Sf)

        # println("z: ", size(z))
        vi = z[:,i] - zpi
        vi[2] = mpi_to_pi(vi[2])

        # Compute proposal covariance and mean
        P = inv(Hvi' * Sfi * Hvi + inv(P))

        x += P * Hvi' * Sfi * vi
    end

    # Get one sample from the Gaussian proposal distribution
    pose_sample = rand_mvn(x, P, 1)

    # Update sample weight: w *= p(z|xk) p(xk|xk-1) / proposal
    like = observation_likelihood_given_pose(particle, z, feature_ids, R)

    prior = eval_mvn(pose_delta(particle.pose, pose_sample), particle.pcov)
    proposal =  eval_mvn(pose_delta(x, pose_sample), P)
    w = particle.weight * like * prior / proposal
    return x, P, w[1,1]
end


"""
For FastSLAM, p(z|x) requires the map part to be marginalised from p(z|x,m)
Parameters:
particle: instance of a Particle
z: Observation array with columns = [range; angle]
tags: Identity of landmarks
R: 2x2 observation covariance matrix

Returns:
w: likelihood of observation given pose
"""
function observation_likelihood_given_pose(particle::Particle, z, tags, R)
    w = 1.0
    for i = 1:length(tags)
        zp, Hv, Hf, Sf = jacobian(particle, tags[i], R)
        v = z[:,i] - zp
        v[2] = mpi_to_pi(v[2])
        w *= eval_mvn(reshape(v, length(v), 1), Sf)
    end
    return w
end


"""
Compute innovation between two pose estimates, with heading angle in [-pi,pi]

Parameters:
pose1: first [x,y,phi] vector
pose1: second [x,y,phi] vector

Returns:
delta: difference
"""
function pose_delta(pose1, pose2)
    delta = pose1 - pose2
    for i = 1:size(delta, 2)
        delta[3,i] = mpi_to_pi(delta[3,i])
    end
    return delta
end


"""
Having selected a new pose from the proposal distribution, this pose is assumed
perfect. Each feature update is computed independently and without pose
uncertainty. The particle is modified in-place.

Parameters:
particle: instance of a Particle
z: Observation array with columns = [range; angle]
tags: Identity of landmarks
R: 2x2 observation covariance matrix

Returns:
nothing
"""
function update_feature_states!{T, U<:Integer}(particle::Particle{T},
                                               z::Matrix{T},
                                               feature_ids::Vector{U},
                                               R::Matrix{T})
    for i = 1:length(feature_ids)
        zp, Hv, Hf, Sf = jacobian(particle, i, R)
        v = z[:,i] - zp
        v[2] = mpi_to_pi(v[2])

        fx, fP = particle.features[:,i], particle.fcovs[:,:,i]
        fx, fP = kalman_cholesky_update(fx, fP, v, R, Hf)
        particle.features[:,i], particle.fcovs[:,:,i] = fx, fP
    end
    return nothing
end


function add_features!{T}(particle::Particle{T}, z::Matrix{T}, R::Matrix{T})
    n_obs = size(z, 2)
    new_features = zeros(2, n_obs)
    new_fcovs = zeros(2, 2, n_obs)

    for i = 1:n_obs
        r, b = z[:,i]  # Range and bearing
        phi = particle.pose[3]
        s, c = sin(phi + b), cos(phi + b)

        new_features[:,i] = [particle.pose[1] + r*c;
                             particle.pose[2] + r*s]
        G = [c -r*s; s r*c]
        new_fcovs[:,:,i] = G*R*G'
    end

    # Update the particle
    particle.features = hcat(particle.features, new_features)
    particle.fcovs = cat(3, particle.fcovs, new_fcovs)
end
