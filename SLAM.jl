VERSION >= v"0.4.0" && __precompile__(true)

module SLAM

export
    # Types
    SlamState,
    Vehicle,
    
    # Functions
    ## common.jl:
    mpi_to_pi,
    frame_transform,
    ellipse,
    compute_landmark_ellipses,
    laser_lines,
    augment!,
    ## vehicle.jl:
    step_vehicle!,
    steer!,
    ## ekf.jl:
    ekf_predict!,
    ekf_predict!,
    batch_update!,
    ## data-association.jl:
    associate,
    compute_association,
    ## observation.jl:
    observe_model

include("common.jl")
include("vehicle.jl")
include("observation.jl")
include("ekf.jl")
include("data-association.jl")

end # SLAM