## common.jl: Core types and shared functions ##

## Core Types ##

# type SlamState{T<:Real}
#     x::Vector{T}
#     cov::Matrix{T}
#     # landmarks?
# end

abstract SlamState

# Extended Kalman Filter SLAM state and covariance
type EKFSlamState{T<:Real} <: SlamState
    x::Vector{T}                       # Joint pose + feature state
    cov::Matrix{T}                     # Joint pose + feature cov matrix
end

# Particle Filter SLAM state and covariance
type PFSlamState{T<:Real} <: SlamState
    pose::Vector{T}                    # Inferred vehicle pose e.g. [x, y, phi]
    features::Matrix{T}                # Columns are feature/landmark positions
    pcov::Matrix{T}                    # Pose covariance matrix
    fcov::Matrix{T}                    # Feature/landmark cov matrix
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
