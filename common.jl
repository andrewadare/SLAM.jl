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


function ellipse(x, P, nsigma, nsegs)
    # Make a single 2D n-sigma Gaussian contour centered at x with axes given
    # by covariance matrix P

    # Approximate smooth ellipse with nsegs line segments (nsegs + 1 points)
    phi = 0:2*pi/nsegs:2*pi

    nsigma*sqrtm(P)*[cos(phi)'; sin(phi)'] .+ x[1:2]
end


function compute_landmark_ellipses(x, P)
    nsigma = 2 # Size of ellipse
    nsegs = 12 # Number of line segments to use

    # Number of observed features/landmarks
    nf = floor(Int, (length(x) - 3)/2)

    # Allocate rows for x,y and columns for npoints/ellipse times # ellipses
    ellipses = Array{Float64}(2*nf, nsegs + 1)

    for i = 1:nf
        j = (2*i + 2):(2*i + 3)

        ell = ellipse(x[j], P[j,j], nsigma, nsegs)
        ellipses[2i-1:2i, :] = ellipse(x[j], P[j,j], nsigma, nsegs)
    end
    ellipses
end


function laser_lines(z,x)
    # Return array of line segments for laser range-bearing measurements.
    # Columns contain vehicle and feature positions [vx; vy; fx; fy]
    len = size(z, 2)
    lines = Array{Float64}(4, len)
    lines[1,:] = zeros(1, len) + x[1]
    lines[2,:] = zeros(1, len) + x[2]
    lines[3:4,:] = frame_transform([z[1,:].*cos(z[2,:]); z[1,:].*sin(z[2,:])], x)
    lines
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

function observe_model(x::AbstractVector, idf::Int)
    # INPUTS:
    #   x - state vector
    #   idf - index of feature order in state
    #
    # OUTPUTS:
    #   z - predicted observation
    #   H - observation Jacobian
    #
    # Given a feature index (ie, the order of the feature in the state vector),
    # predict the expected range-bearing observation of this feature and its Jacobian.

    Nxv = 3 # number of vehicle pose states
    fpos = Nxv + idf*2 - 1 # position of xf in state
    H = zeros(2, length(x))

    # auxiliary values
    dx = x[fpos] - x[1] 
    dy = x[fpos + 1] - x[2]
    d2 = dx^2 + dy^2
    d = sqrt(d2)
    xd = dx/d
    yd = dy/d
    xd2 = dx/d2
    yd2 = dy/d2
    phi = x[3]

    # predict z
    z = [d; atan2(dy,dx) - phi]

    # calculate H
    H[:,1:3]         = [-xd -yd 0; yd2 -xd2 -1]
    H[:,fpos:fpos+1] = [ xd  yd;  -yd2  xd2]
 
    z, H
end
