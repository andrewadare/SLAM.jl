function associate_known!(z, tags, da_list, nf)
    """
    Parameters:

    z: Observation array with columns = [range; angle]
    tags: Identity of landmarks
    da_list: data association list
    nf: total number of features observed so far

    Returns:
    zf: list of measurements corresponding to known features
    idf: list of feature IDs corresponding to known features
    zn: list of new measurements

    Updates da_list in-place.
    """
    zf = Matrix{eltype(z)}(size(z, 1), 0)  # Already-seen feature positions
    zn = Matrix{eltype(z)}(size(z, 1), 0)  # New feature positions
    idf = Vector{eltype(tags)}(0)  # IDs of already-seen features
    idn = Vector{eltype(tags)}(0)  # New feature IDs

    # Find associations (zf) and new features (zn)
    for i = 1:length(tags)
        ii = tags[i]

        if da_list[ii] == 0
            # Observed a new feature - append the measurement
            zn = [zn z[:,i]]
            idn = push!(idn, ii)
        else
            zf = [zf z[:,i]]
            idf = push!(idf, da_list[ii])
        end
    end

    # Add new feature IDs to data association list
    da_list[idn] = nf + (1:size(zn, 2))

    return zf, idf, zn
end


function associate(state::SlamState, z, R, gate1, gate2)
    # Simple gated nearest-neighbour data-association. No clever feature
    # caching tricks to speed up association, so computation is O(N), where
    # N is the number of features in the state.

    x = state.x
    P = state.cov
    n_observables = size(z, 1)

    zf = Array{AbstractFloat}(n_observables, 0)
    zn = Array{AbstractFloat}(n_observables, 0)
    idf = Array{Int}(1, 0)

    Nxv = 3 # number of vehicle pose components [x, y, phi]
    Nf = round(Int, (length(x) - Nxv)/2) # number of features already in map

    # linear search for nearest-neighbour, no clever tricks (like a quick
    # bounding-box threshold to remove distant features; or, better yet,
    # a balanced k-d tree lookup). TODO: implement clever tricks.
    for i = 1:size(z,2)
        jbest = 0
        nbest = Inf
        outer = Inf

        # search for neighbours
        for j = 1:Nf
            nis, nd = compute_association(x, P, z[:,i], R, j)
            ingate = 0
            if nis < gate1
                if nd < nbest # if within gate, store nearest-neighbour
                    ingate = 1
                end
            end
            if ingate == 1 # if within gate, store nearest-neighbour
                nbest = nd
                jbest = j
            elseif nis < outer # else store best nis value
                outer = nis
            end
        end
        # add nearest-neighbour to association list
        if jbest != 0
            zf = [zf  z[:,i]]
            idf = [idf jbest]
        elseif outer > gate2 # z too far to associate, but far enough to be a new feature
            zn = [zn z[:,i]]
        end
    end
    zf, idf, zn
end

function compute_association(x, P, z, R, idf)
    # Return normalised innovation squared (Mahalanobis distance) and
    # normalised distance
    zp, H = predict_observation(x, idf)
    v = z - zp
    v[2] = mpi_to_pi(v[2])
    S = H*P*H' + R
    nis = dot(v, inv(S)*v)
    nd = nis + log(det(S))
    nis, nd
end
