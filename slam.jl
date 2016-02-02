VERSION >= v"0.4.0" && __precompile__(true)

module SLAM

export
    # Types
    Scene,
    SlamState,
    EKFSlamState,
    PFSlamState,
    Vehicle,
    
    # Functions
    initial_pose,
    mpi_to_pi,
    frame_transform,
    step_vehicle!,
    steer!,
    predict,
    update,
    add_features,
    associate,
    compute_association,
    predict_observation


include("common.jl")
include("vehicle.jl")
include("ekf.jl")
include("data-association.jl")

end # SLAM