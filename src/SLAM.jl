__precompile__(true)

module SLAM

export
    # Types
    Scene,
    SlamState,
    EKFSlamState,
    PFSlamState,
    Vehicle,
    SimData,

    # Functions
    get_waypoints,
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
    predict_observation,
    ellipse,
    compute_landmark_ellipses,
    laser_lines


include("common.jl")
include("ekf.jl")
include("data-association.jl")

end # SLAM