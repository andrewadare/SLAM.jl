# SLAM simulation using extended Kalman filter. Inspired by Matlab implementation
# by T. Bailey et al (https://openslam.org/bailey-slam.html).

using HttpServer
using WebSockets
import JSON

# Here are three options to make the SLAM module (or any other) available to your code:
#
# 1. Include it directly: `include("path/to/SLAM.jl/src/SLAM.jl")`
# 2. Add it to Julia's LOAD_PATH environment variable: if `module_dir = "/path/to/SLAM.jl"`,
#    add a line like `module_dir in LOAD_PATH || push!(LOAD_PATH, module_dir)` to your code,
#    your REPL session, or to `~/.juliarc.jl`.
# 3. Use SLAM.jl as a package: Pkg.clone("url/to/SLAM.jl")
include("../../src/SLAM.jl")

using SLAM

include("../sim-utils.jl")
include("../ekfslam-sim.jl")
include("../fastslam-sim.jl")

# TODO: make this settable as a command-line argument, or selectable on the page
const SLAM_ALGORITHM = "fastslam"  # "ekf" or "fastslam"

function start(simdata::SimData,
               scene::Scene,
               vehicle::Vehicle,
               state::SlamState,
               monitor::Function,
               client::WebSockets.WebSocket)
    simdata.paused = false
    @async sim!(simdata, scene, vehicle, state, monitor,
        [simdata, scene, vehicle, state, client])
end


"""
Callback passed to simulation providing access to simulation state during
runtime. This method writes messages to a WebSocket client for browser
visualization.
"""
function monitor(simdata::SimData,
                 scene::Scene,
                 vehicle::Vehicle,
                 state::EKFSlamState,
                 client::WebSockets.WebSocket)

    n = scene.nsteps
    tt, st = scene.true_track, scene.slam_track
    pose = state.x[1:3]

    # Send latest pose information
    d = Dict("ideal" => Dict("x" => tt[1, n], "y" => tt[2, n], "phi" => tt[3, n]),
             "slam"  => Dict("x" => st[1, n], "y" => st[2, n], "phi" => st[3, n]))
    send_json("tracks", d, client)

    # Send SLAM state
    # TODO: is this used?
    d = Dict("pose" => pose, "cov" => state.cov)
    send_json("state", d, client)

    # Send line endpoints for lidar beams from vehicle to observed feature
    if simdata.state_updated && simdata.nz > 0
        lines = laser_lines(simdata.z[:,1:simdata.nz], state.x[1:3])

        nlines = size(lines, 2)
        for j = 1:nlines
            vx, vy, zx, zy = lines[1:4, j]
            @assert inbounds(vx, vy, scene) "vx, vy: $vx $vy"
            @assert inbounds(zx, zy, scene) "zx, zy: $zx $zy"
        end

        send_json("lidar", dict_array(lines, ["x1", "y1", "x2", "y2"]), client)

        # Write feature uncertainty ellipses
        if length(state.x) > 3
            ellipses = feature_ellipses(state.x, state.cov)
            ellipse_keys = ["cx", "cy", "rx", "ry", "phi"]
            send_json("feature-ellipses", dict_array(ellipses, ellipse_keys), client)
        end
    end

    # Write uncertainty ellipse for vehicle position
    let
        l,u = eig(state.cov[1:2, 1:2])
        vehicle_ellipse = [pose' sqrt.(l)' atan2(u[2,1], u[1,1])]'
        ellipse_keys = ["cx", "cy", "vehicle_phi", "rx", "ry", "phi"]
        send_json("vehicle-ellipse", dict_array(vehicle_ellipse, ellipse_keys), client)
    end

    return
end


"""
Callback passed to simulation providing access to simulation state during
runtime. This method writes messages to a WebSocket client for browser
visualization.
"""
function monitor(simdata::SimData,
                 scene::Scene,
                 vehicle::Vehicle,
                 state::PFSlamState,
                 client::WebSockets.WebSocket)

    n = scene.nsteps
    tt, st = scene.true_track, scene.slam_track
    pose = state.x

    # Send latest pose information
    d = Dict("ideal" => Dict("x" => tt[1, n], "y" => tt[2, n], "phi" => tt[3, n]),
             "slam"  => Dict("x" => st[1, n], "y" => st[2, n], "phi" => st[3, n]))
    send_json("tracks", d, client)

    # Send vehicle particle positions. `phi` excluded for now
    veh_particles = reshape(vcat([p.pose for p in state.particles]...),
        (length(pose), state.nparticles))
    send_json("vehicle-particles", dict_array(veh_particles, ["x", "y"]), client)

    # Send line endpoints for lidar beams from vehicle to observed feature
    if simdata.state_updated && simdata.nz > 0
        lines = laser_lines(simdata.z[:,1:simdata.nz], state.x[1:3])

        nlines = size(lines, 2)
        for j = 1:nlines
            vx, vy, zx, zy = lines[1:4, j]
            @assert inbounds(vx, vy, scene) "vx, vy: $vx $vy"
            @assert inbounds(zx, zy, scene) "zx, zy: $zx $zy"
        end

        send_json("lidar", dict_array(lines, ["x1", "y1", "x2", "y2"]), client)

        # Write feature uncertainty ellipses
        if length(state.x) > 3
            ellipses = feature_ellipses(state.x, state.cov)
            # The 5 parameters for a rotated covariance ellipse
            ellipse_keys = ["cx", "cy", "rx", "ry", "phi"]
            send_json("feature-ellipses", dict_array(ellipses, ellipse_keys), client)
        end

        lm_particles = hcat([p.features for p in state.particles]...)
        send_json("landmark-particles", dict_array(lm_particles, ["x", "y"]), client)
    end

    # Write uncertainty ellipse for vehicle position
    let
        l,u = eig(state.cov[1:2, 1:2])
        vehicle_ellipse = [pose' sqrt.(l)' atan2(u[2,1], u[1,1])]'
        ellipse_keys = ["cx", "cy", "vehicle_phi", "rx", "ry", "phi"]
        send_json("vehicle-ellipse", dict_array(vehicle_ellipse, ellipse_keys), client)
    end

    return
end


function feature_ellipses(x, cov)
    # Number of observed features/landmarks
    nf = floor(Int, (length(x) - 3)/2)

    # Allocate rows for x,y and columns for npoints/ellipse times # ellipses
    ellipses = Array{Float64}(5, nf)

    for i = 1:nf
        j = (2*i + 2):(2*i + 3)
        l,u = eig(cov[j,j])
        ellipses[:, i] = [x[j]; sqrt.(l); atan2(u[2,1], u[1,1])]
    end
    ellipses
end


"""
Write JSON-formatted message to websocket client.
Recipients expect the schema defined here, so modify with care.
"""
function send_json(name::AbstractString, data::Any, client::WebSockets.WebSocket)
    msg = Dict{AbstractString, Any}()
    msg["type"] = name
    msg["data"] = data
    msg["timestamp"] = time()
    write(client, JSON.json(msg))
end


"""
Create array of Dicts from Matrix `a` using the supplied list of `keys`.
Example:
        julia> a = rand(1:10, 2, 3)
        2x3 Array{Int64,2}:
         10  5  8
          5  3  6

        julia> keys = ["a", "b"]
        2-element Array{String,1}:
         "a"
         "b"

        julia> dict_array(a, keys)
        3-element Array{Dict{String,Int64},1}:
         Dict("b"=>5,"a"=>10)
         Dict("b"=>3,"a"=>5)
         Dict("b"=>6,"a"=>8)
"""
function dict_array{T,S}(a::Array{T}, keys::Vector{S})
    assert(length(keys) <= size(a, 1))

    n = size(a, 2)
    da = Vector{Dict{S, T}}(n)

    # Loop over columns of a
    for j = 1:n
        d = Dict{S,T}()

        for (i, key) in enumerate(keys)
            d[key] = a[i,j]
        end
        da[j] = d
    end
    return da
end


wsh = WebSocketHandler() do req, client
    println("Handling WebSocket client")
    println("    client.id: ",         client.id)
    println("    client.socket: ",     client.socket)
    println("    client.is_closed: ",  client.is_closed)
    println("    client.sent_close: ", client.sent_close)

    # simdata contains simulation state, updated in real time.
    # It is exposed for monitoring, debugging, and flexible visualization.
    scene, simdata = sim_setup(10, "../course1.txt")

    if SLAM_ALGORITHM == "fastslam"
        state = PFSlamState(100, initial_pose(scene))
    elseif SLAM_ALGORITHM == "ekf"
        # First three elements comprise the SLAM vehicle pose; state is augmented as
        # new landmarks are observed. Covariance matrix is also augmented during simulation.
        state = EKFSlamState{Float64}(zeros(3), zeros(3, 3))
    end

    vehicle = default_vehicle()
    vehicle.pose = initial_pose(scene)

    # TODO improve this
    if typeof(state) == EKFSlamState{Float64}
        state.x = initial_pose(scene)
    end

    while true

        # Read string from client, decode, and parse to Dict
        msg = JSON.parse(String(copy(read(client))))

        if haskey(msg, "text") && msg["text"] == "ready"
            println("Received update from client: ready")
            send_json("waypoints", dict_array(scene.waypoints, ["x", "y"]), client)
            send_json("landmarks", dict_array(scene.landmarks, ["x", "y"]), client)
        end

        if haskey(msg, "text") && msg["text"] == "start"
            start(simdata, scene, vehicle, state, monitor, client)
        end

        if haskey(msg, "text") && msg["text"] == "reset"
            scene.nsteps = 0
            scene.true_track *= 0
            scene.slam_track *= 0

            vehicle.waypoint_id = 1
            vehicle.pose = initial_pose(scene)

            # reset!(state)

            # TODO improve this
            if typeof(state) == EKFSlamState{Float64}
                state.x = initial_pose(scene)
                state.cov = zeros(3, 3)
            end

            simdata.state_updated = false
            simdata.paused = false
        end

        if haskey(msg, "text") && msg["text"] == "pause"
            simdata.paused = !simdata.paused
            if !simdata.paused
                start(simdata, scene, vehicle, state, monitor, client)
            end
        end

    end
end


"""
Serve page(s) and supporting files over HTTP. Assumes the server is started
from the location of this script. Searches through server root directory and
subdirectories recursively for the requested resource.
"""
httph = HttpHandler() do req::Request, res::Response

    # Handle case where / means index.html
    if req.resource == "/"
        println("serving ", req.resource)
        return Response(readstring("index.html"))
    end

    # Serve requested file if found, else return a 404.
    for (root, dirs, files) in walkdir(".")
        for file in files
            file = replace(joinpath(root, file), "./", "")
            if startswith("/$file", req.resource)
                println("serving ", file)
                return Response(open(readstring, file))
            end
        end
    end
    return Response(404)
end

httph.events["error"]  = (client, err) -> println(err)
httph.events["listen"] = (port) -> println("Listening on $port...")

# Instantiate and start a websockets/http server
server = Server(httph, wsh)
println("Starting WebSocket server.")
run(server, 8000)
