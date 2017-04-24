"""
predict(state::EKFSlamState, vehicle::Vehicle, Q::AbstractMatrix, dt::AbstractFloat)

Predict SLAM pose and covariance matrix according to equations of motion for vehicle.
Q is the covariance matrix for speed and steering angle gamma.
dt is the timestep
"""
function predict_pose(state::EKFSlamState, vehicle::Vehicle, Q::AbstractMatrix, dt::AbstractFloat)

    x = state.x
    P = state.cov

    # Predicted pose and covariance, as well as Jacobian matrices
    xpred, cov_pred, Gu, Gv = predict_pose(x[1:3], P[1:3, 1:3], vehicle, Q, dt)

    x[1:3], P[1:3,1:3] = xpred, cov_pred

    if size(P, 1) > 3
        P[1:3,4:end] = Gv*P[1:3,4:end]
        P[4:end,1:3] = P[1:3,4:end]'
    end

    return x, P
end


function update(state::EKFSlamState, z, R, idf)
    x = state.x
    P = state.cov

    lenz = size(z,2)
    lenx = length(x)
    H = zeros(2*lenz, lenx)
    v = zeros(2*lenz)
    RR = zeros(2*lenz, 2*lenz)

    for i = 1:lenz
        ii = 2*i + (-1:0)
        zp, H[ii,:] = predict_observation(x, idf[i])

        v[ii] = [z[1,i] - zp[1]; mpi_to_pi(z[2,i] - zp[2])]
        RR[ii,ii] = R
    end

    x, P = kalman_cholesky_update(x, P, v, RR, H)

    return x, P
end


"""
Add new feature/landmark measurements z and return augmented state
vector x and covariance matrix P
"""
function add_features(state::EKFSlamState, z, R)

    x = state.x
    P = state.cov
    phi = x[3]

    for i = 1:size(z, 2)
        len = length(x)

        # Range and bearing measurement i
        r, b = z[1,i], z[2,i]

        s, c = sin(phi + b), cos(phi + b)

        # augment x
        x = [x; x[1] + r*c; x[2] + r*s]

        # jacobians
        Gv = [1 0 -r*s; 0 1 r*c]
        Gz = [c -r*s; s r*c]

        # Augment P
        # TODO: Implement resizing more efficiently(?) For ideas, see
        # https://groups.google.com/forum/#!topic/julia-users/B4OUYPFM5L8
        P = hcat(P, zeros(size(P, 1), 2))
        P = vcat(P, zeros(2, size(P, 2)))

        rng = len+1:len+2
        P[rng,rng] = Gv*P[1:3,1:3]*Gv' + Gz*R*Gz' # feature cov
        P[rng,1:3] = Gv*P[1:3,1:3] # vehicle to feature xcorr
        P[1:3,rng] = P[rng,1:3]'
        if len > 3
            rnm = 4:len
            P[rng,rnm] = Gv*P[1:3,rnm] # map to feature xcorr
            P[rnm,rng] = P[rng,rnm]'
        end
    end
    x, P
end
