__precompile__(true)

module SLAM

export
    # Types
    EKFSlamState,
    Scene,
    SimData,
    SlamState,
    PFSlamState,
    Vehicle,

    # Functions
    add_features,
    associate,
    compute_association,
    compute_landmark_ellipses,
    ellipse,
    fuse_heading_measurement,
    local_to_global,
    get_waypoints,
    inbounds,
    initial_pose,
    laser_lines,
    mpi_to_pi,
    predict_pose,
    predict_observation,
    reset!,
    steer!,
    step_vehicle!,
    update


include("common.jl")
include("ekf.jl")
include("fastslam.jl")
include("data-association.jl")

end # SLAM