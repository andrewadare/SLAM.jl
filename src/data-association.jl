function associate(state::EKFSlamState, z, R, gate1, gate2)
    #
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
    # Return normalised innovation squared (ie, Mahalanobis distance) and
    # normalised distance
    zp, H = predict_observation(x, idf)
    v = z - zp
    v[2] = mpi_to_pi(v[2])
    S = H*P*H' + R
    nis = dot(v, inv(S)*v)
    nd = nis + log(det(S))
    nis, nd
end

function associate_known(state::PFSlamState, z::AbstractMatrix, tags::AbstractVector)

    # Number of observed features, i.e. columns in features matrix
    nfeatures = size(state.particles[1].features, 2)

    zf = []
    zn = []
    idf = []
    idn = []
    table = copy(state.da_table)

    for i in 1:length(tags)
        tag = tags[i]
        observation = z[:,i]

        # If this tag (feature) is new, append it as a column to the list of new observations
        if table[tag] == 0
            zn = [zn observation]
            idn = [idn; tag]
        else
            zf = [zf observation]
            idf = [idf; table[tag]]
        end
    end

    # Create IDs for new features and put in lookup table
    n_new_features = size(zn, 2)
    new_feature_ids = nfeatures + collect(1:n_new_features)
    table[idn] = new_feature_ids

    zf, idf, zn, table
end
