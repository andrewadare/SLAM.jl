
module_dir = "$(pwd())/.."
module_dir in LOAD_PATH || push!(LOAD_PATH, module_dir)

include("ekfslam-sim.jl")

function monitor(scene::Scene, vehicle::Vehicle, state::SlamState)
    println(scene.nsteps)
end

scene, vehicle, state = ekfsim_setup(10, "course1.txt")
sim(scene, vehicle, state, monitor, [scene, vehicle, state])
