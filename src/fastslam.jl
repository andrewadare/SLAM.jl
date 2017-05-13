
function sample_proposal(particle::Particle, z, tags, R)
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
    x = copy(particle.pose)
    P = copy(particle.pcov)

    for i = 1:length(tags)
        j = tags[i]
        zpi, Hvi, Hfi, Sf = jacobians(particle, j, R)
        Sfi = inv(Sf)

        vi = z[:,i] - zpi
        vi[2] = mpi_to_pi(vi[2])

        # Compute proposal covariance and mean
        P = inv(Hvi' * Sfi * Hvi + inv(P))

        x += P * Hvi' * Sfi * vi
    end

    # Get one sample from the Gaussian proposal distribution
    pose_sample = rand_mvn(x, P, 1)

    # Update sample weight: w *= p(z|xk) p(xk|xk-1) / proposal
    like = observation_likelihood_given_pose(particle, z, tags, R)

    prior = eval_mvn(pose_delta(particle.pose, pose_sample), particle.pcov)
    proposal =  eval_mvn(pose_delta(x, pose_sample), P)
    w = particle.w * like * prior / proposal

    return x, P, w
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
        zp, Hv, Hf, Sf = jacobians(particle, tags[i], R)
        v = z[:,i] - zp
        v[2] = mpi_to_pi(v[2])

        w *= eval_mvn(v, Sf)
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
    delta[3,:] = mpi_to_pi[delta[3,:]]
    return delta
end

