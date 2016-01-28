VERSION >= v"0.4.0" && __precompile__(true)

module SLAM

export
    # Types
    SlamState,
    Vehicle,
    
    # Functions
    mpi_to_pi,
    frame_transform,
    augment!,
    step_vehicle!,
    steer!,
    ekf_predict!,
    ekf_predict!,
    batch_update!,
    associate,
    compute_association,
    predict_observation


include("common.jl")
include("vehicle.jl")
include("ekf.jl")
include("data-association.jl")

end # SLAM