
module_dir = "$(pwd())/.."
module_dir in LOAD_PATH || push!(LOAD_PATH, module_dir)

include("ekfslam-sim.jl")

function monitor(simdata::SimData)
    println(simdata.scene.nsteps)
end

# Global object containing simulation state, updated in real time. 
# It is exposed for monitoring, debugging, and flexible visualization.
simdata = ekfsim_setup(10, "course1.txt")

sim!(simdata, monitor, [simdata])