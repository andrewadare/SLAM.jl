function ekf_predict!(state::SlamState, vehicle::Vehicle, Q::AbstractMatrix, dt::AbstractFloat)
    #  x, P - SLAM state vector and covariance matrix
    #  speed, gamma - control inputs: vehicle speed and steer angle
    #  Q - covariance matrix for speed and gamma
    #  wheelbase - vehicle wheelbase
    #  dt - timestep
    #
    #  x,P are modified in-place (!) to be the predicted state and covariance

    # Abbreviations
    phi = state.x[3]
    P = state.cov
    g = vehicle.measured_gamma
    v = vehicle.measured_speed
    w = vehicle.wheelbase

    # Cached intermediates
    s = sin(g + phi)
    c = cos(g + phi)
    vts = v*dt*s
    vtc = v*dt*c

    # Jacobians   
    Gv = [1 0 -vts;
          0 1 vtc;
          0 0 1]
    Gu = [dt*c -vts;
          dt*s  vtc;
          dt*sin(g)/w v*dt*cos(g)/w]
  
    # Predict covariance
    P[1:3,1:3] = Gv*P[1:3,1:3]*Gv' + Gu*Q*Gu'
    if size(P, 1) > 3
        P[1:3,4:end] = Gv*P[1:3,4:end]
        P[4:end,1:3] = P[1:3,4:end]'
    end
    state.cov = P

    # Predict pose
    state.x[1:3] = [state.x[1] + vtc; 
                    state.x[2] + vts;
                    mpi_to_pi(phi + v*dt*sin(g)/w)]
end


function ekf_predict!(x, P, speed, gamma, Q, wheelbase, dt)
    #  x, P - SLAM state vector and covariance matrix
    #  speed, gamma - control inputs: vehicle speed and steer angle
    #  Q - covariance matrix for speed and gamma
    #  wheelbase - vehicle wheelbase
    #  dt - timestep
    #
    #  x,P are modified in-place (!) to be the predicted state and covariance
    
    phi = x[3]

    # Cached intermediates
    s = sin(gamma + phi)
    c = cos(gamma + phi)
    vts = speed*dt*s
    vtc = speed*dt*c

    # Jacobians   
    Gv = [1 0 -vts;
          0 1 vtc;
          0 0 1]
    Gu = [dt*c -vts;
          dt*s  vtc;
          dt*sin(gamma)/wheelbase speed*dt*cos(gamma)/wheelbase]
  
    # Predict covariance
    P[1:3,1:3] = Gv*P[1:3,1:3]*Gv' + Gu*Q*Gu'
    if size(P, 1) > 3
        P[1:3,4:end] = Gv*P[1:3,4:end]
        P[4:end,1:3] = P[1:3,4:end]'
    end    

    # Predict state (the pose component)
    x[1:3] = [x[1] + vtc; 
              x[2] + vts;
              mpi_to_pi(phi + speed*dt*sin(gamma)/wheelbase)]
    x, P
end


function batch_update!(state::SlamState, z, R, idf)
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

    # Calculate the KF (or EKF) update given the prior state [x,P]
    # the innovation [v,RR] and the (linearised) observation model H.
    # The result is calculated using Cholesky factorisation, which
    # is more numerically stable than a naive implementation.
    PHt = P*H'
    S = H*PHt + RR
    S = (S + S')*0.5 # make symmetric
    C = inv(chol(S)) # triangular matrix
    W1 = PHt*C
    W = W1*C'

    state.x += W*v 
    state.cov -= W1*W1'

end
