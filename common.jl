## common.jl: Core types and shared functions ##

## Core Types ##

type SlamState{T<:Real}
    x::Vector{T}
    cov::Matrix{T}
    # landmarks?
end


## Shared functions ##

function mpi_to_pi(phi)
    if phi > pi
        return phi - 2*pi
    end
    if phi < -pi
        return phi + 2*pi
    end
    phi
end


function frame_transform(p, b)

    q = zeros(p)
        
    # Rotate and translate
    R = [cos(b[3]) -sin(b[3]); sin(b[3]) cos(b[3])]
    q[1:2,:] = R*p[1:2,:] .+ b[1:2]

    # if p is a pose and not a point
    if size(p,1)==3
        q[3,:] = mpi_to_pi(p[3,:] + b[3])
    end
    q
end


"""
Grow state vector x and covariance matrix P with landmark measurements
"""
function augment!(state::SlamState, z, R)
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
    state.x = x
    state.cov = P
    return
end


function augment(x, P, z, R)
    # Grow state vector x and covariance matrix P with landmark measurements

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

"""
Predict the expected range-bearing observation z (and Jacobian H) of 
feature/landmark idf in joint pose + landmark state vector x
"""
function predict_observation(x::AbstractVector, idf::Int)

    # Map feature index idf to position fpos in x
    pose_length = 3
    fpos = pose_length + idf*2 - 1

    # Distance from vehicle to landmark
    dx = x[fpos] - x[1] 
    dy = x[fpos + 1] - x[2]
    d2 = dx^2 + dy^2
    d = sqrt(d2)

    # Predict [range, bearing] observation z
    z = [d; atan2(dy,dx) - x[3]]

    # Calculate Jacobian H for pose and z
    xd = dx/d
    yd = dy/d
    xd2 = dx/d2
    yd2 = dy/d2

    H = zeros(2, length(x))
    H[:,1:3]         = [-xd -yd 0; yd2 -xd2 -1]
    H[:,fpos:fpos+1] = [ xd  yd;  -yd2  xd2]
 
    z, H
end
