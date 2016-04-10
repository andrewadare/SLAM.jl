

"""
predict(state::PFSlamState, vehicle::Vehicle, dt::AbstractFloat)

Predict SLAM pose and covariance matrix according to equations of motion for vehicle.
dt is the timestep
"""
function predict(state::PFSlamState, vehicle::Vehicle, dt::AbstractFloat)

    particles = copy(state.particles)

    for i in 1:length(particles)
        predict!(particles[i].pose, vehicle, dt)
    end

    particles
end


function predict!(pose::Pose2D, vehicle::Vehicle, dt::AbstractFloat)
    vt = vehicle.measured_speed*dt
    g = vehicle.measured_gamma
    w = vehicle.wheelbase
    pose.x += vt*cos(phi + g)
    pose.y += vt*sin(phi + g)
    pose.phi = mpi_to_pi(pose.phi + vt*sin(g)/w)
end


"""
Compute proposal distribution, draw a sample, and update particle weight.
"""
function sample_proposal(particle::Particle, z::AbstractMatrix, idf, R)

    #
    for i in 1:length(idf)

    end
end


# idf is an array of indices for observed features
function compute_jacobians(particle::Particle, idf::AbstractVector, R)

    # xv = particle.xv
    # xf= particle.xf(:,idf)
    # Pf= particle.Pf(:,:,idf)

    # TODO pick up here
    for i in 1:length(idf)
        dx = particle.features[1, i] - particle.pose.x
        dy = particle.features[2, i] - particle.pose.y

        d2 = dx^2 + dy^2
        d =  sqrt(d2)

        # Predicted observation
        zp(:,i)= [d; pi_to_pi(atan2(dy,dx) - xv(3))]

        # Jacobian wrt vehicle states
        Hv(:,:,i)= [-dx/d,  -dy/d,   0;
                     dy/d2, -dx/d2, -1]

        # Jacobian wrt feature states
        Hf(:,:,i)= [ dx/d,   dy/d;
                    -dy/d2,  dx/d2]

        # Innovation covariance of 'feature observation given the vehicle'
        Sf(:,:,i)= Hf(:,:,i) * Pf(:,:,i) * Hf(:,:,i)' + R
    end
    zp, Hv, Hf, Sf
end