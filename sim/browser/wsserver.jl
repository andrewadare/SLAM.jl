using HttpServer
using WebSockets
import JSON

include("../ekfslam-sim.jl")

# Global object containing simulation state, updated in real time.
# It is exposed for monitoring, debugging, and flexible visualization.
simdata = ekfsim_setup(10, "../course1.txt")

function start(simdata::SimData, monitor::Function, client::WebSockets.WebSocket)
    simdata.paused = false
    @async sim!(simdata, monitor, [simdata, client])
end


"""
Callback passed to simulation providing access to simulation state during
runtime. This method writes messages to a WebSocket client for browser
visualization.
"""
function monitor(simdata::SimData, client::WebSockets.WebSocket)

    n = simdata.scene.nsteps
    tt, st = simdata.scene.true_track, simdata.scene.slam_track
    pose = simdata.state.x[1:3]

    # The 5 parameters for a rotated covariance ellipse

    # Send latest pose information
    d = Dict("ideal" => Dict("x" => tt[1, n], "y" => tt[2, n], "phi" => tt[3, n]),
             "slam"  => Dict("x" => st[1, n], "y" => st[2, n], "phi" => st[3, n]))
    send_json("tracks", d, client)

    # Send SLAM state
    d = Dict("pose" => pose, "cov" => simdata.state.cov)
    send_json("state", d, client)

    # Send line endpoints for lidar beams from vehicle to observed feature
    if simdata.state_updated && simdata.nz > 0
        lines = laser_lines(simdata.z[:,1:simdata.nz], simdata.state.x[1:3])

        nlines = size(lines, 2)
        for j = 1:nlines
            vx, vy, zx, zy = lines[1:4, j]
            @assert inbounds(vx, vy, simdata.scene) "vx, vy: $vx $vy"
            @assert inbounds(zx, zy, simdata.scene) "zx, zy: $zx $zy"
        end

        send_json("lidar", dict_array(lines, ["x1", "y1", "x2", "y2"]), client)

        # Write feature uncertainty ellipses
        if length(simdata.state.x) > 3
            ellipses = feature_ellipses(simdata.state.x, simdata.state.cov)
            ellipse_keys = ["cx", "cy", "rx", "ry", "phi"]
            send_json("feature-ellipses", dict_array(ellipses, ellipse_keys), client)
        end
    end

    # Write uncertainty ellipse for vehicle position
    let
        l,u = eig(simdata.state.cov[1:2, 1:2])
        vehicle_ellipse = [pose' sqrt(l)' atan2(u[2,1], u[1,1])]'
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
        ellipses[:, i] = [x[j]; sqrt(l); atan2(u[2,1], u[1,1])]
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
function dict_array{T,S}(a::Matrix{T}, keys::Vector{S})
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
    da
end


wsh = WebSocketHandler() do req, client
    println("Handling WebSocket client")
    println("    client.id: ",         client.id)
    println("    client.socket: ",     client.socket)
    println("    client.is_closed: ",  client.is_closed)
    println("    client.sent_close: ", client.sent_close)

    while true

        # Read string from client, decode, and parse to Dict
        msg = JSON.parse(String(copy(read(client))))

        if haskey(msg, "text") && msg["text"] == "ready"
            println("Received update from client: ready")
            send_json("waypoints", dict_array(simdata.scene.waypoints, ["x", "y"]), client)
            send_json("landmarks", dict_array(simdata.scene.landmarks, ["x", "y"]), client)
        end

        if haskey(msg, "text") && msg["text"] == "start"
            start(simdata, monitor, client)
        end

        if haskey(msg, "text") && msg["text"] == "reset"
            simdata.scene.nsteps = 0
            simdata.scene.true_track *= 0
            simdata.scene.slam_track *= 0

            simdata.state.x = initial_pose(simdata.scene)
            simdata.state.cov = zeros(3, 3)

            simdata.vehicle.waypoint_id = 1
            simdata.vehicle.pose = initial_pose(simdata.scene)

            simdata.state_updated = false
            simdata.paused = false
        end

        if haskey(msg, "text") && msg["text"] == "pause"
            simdata.paused = !simdata.paused
            if !simdata.paused
                start(simdata, monitor, client)
            end
        end

    end
end


"""
Serve page(s) and supporting files over HTTP.
Files will not be loaded by browser unless they appear in this list!
Paths are relative to the location of this script (or invocation location?)
"""
httph = HttpHandler() do req::Request, res::Response

    # Handle case where / means index.html
    if req.resource == "/"
        println("serving ", req.resource)
        return Response(readstring("index.html"))
    end

    files = [
    "index.html",
    "js/jquery-2.1.3.min.js",
    "js/d3.min.js",
    "js/wsclient.js"
    ]

    for file in files
        if startswith("/$file", req.resource)
            println("serving /", file)
            return Response(open(readstring, file))
        end
    end

    Response(404)
end

httph.events["error"]  = (client, err) -> println(err)
httph.events["listen"] = (port)        -> println("Listening on $port...")

# Instantiate and start a websockets/http server
server = Server(httph, wsh)
println("Starting WebSocket server.")
run(server, 8000)
