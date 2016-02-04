using HttpServer
using WebSockets
import JSON

module_dir = "$(pwd())/../.."
module_dir in LOAD_PATH || push!(LOAD_PATH, module_dir)

include("../ekfslam-sim.jl")

# Global object containing simulation state, updated in real time.
# It is exposed for monitoring, debugging, and flexible visualization.
simdata = ekfsim_setup(10, "../course1.txt")

function send_json(name::AbstractString, data::Any, client::WebSockets.WebSocket)
    msg = Dict{AbstractString, Any}()
    msg["type"] = name
    msg["data"] = data
    msg["timestamp"] = time()
    write(client, JSON.json(msg))
end

function monitor(simdata::SimData, client::WebSockets.WebSocket)

    n = simdata.scene.nsteps
    tt, st = simdata.scene.true_track, simdata.scene.slam_track

    # Send latest pose information
    d = Dict("ideal" => Dict("x"   => tt[1, n],
                             "y"   => tt[2, n],
                             "phi" => tt[3, n]),
             "slam" => Dict("x"   => st[1, n],
                            "y"   => st[2, n],
                            "phi" => st[3, n]))
    send_json("tracks", d, client)

    # Send SLAM state
    d = Dict("pose" => simdata.state.x[1:3], "cov" => simdata.state.cov)
    send_json("state", d, client)

    # Send observations, but only when just recorded
    if simdata.state_updated && simdata.nz > 0
        nz = simdata.nz
        z = simdata.z[:, 1:nz]
        d = Dict("range" => vec(z[1,:]),
                 "bearing" => vec(z[2,:]),
                 "n_observations" => nz)
        send_json("observations", d, client)
    end

    return
end


"""
Create a JSON message with x,y positions from 2xN array
"""
function xypoints_json(a::AbstractArray, msgtype::AbstractString)

    msg = Dict{AbstractString, Any}()
    msg["type"] = msgtype
    msg["data"] = [Dict(:x => a[1,i], :y => a[2,i]) for i in 1:size(a, 2)]
    msg["timestamp"] = time()

    # Convert Dict to stringified JSON blob
    JSON.json(msg)
end

wsh = WebSocketHandler() do req, client
    println("Handling WebSocket client")
    println("    client.id: ",         client.id)
    println("    client.socket: ",     client.socket)
    println("    client.is_closed: ",  client.is_closed)
    println("    client.sent_close: ", client.sent_close)

    while true

        # Read string from client, decode, and parse to Dict
        msg = JSON.parse(bytestring(read(client)))

        if haskey(msg, "text") && msg["text"] == "ready"
            println("Received update from client: ready")
            write(client, xypoints_json(simdata.scene.waypoints, "waypoints"))
            write(client, xypoints_json(simdata.scene.landmarks, "landmarks"))
        end

        if haskey(msg, "text") && msg["text"] == "start"
            println("Answering request: start")
            # TODO re-initialize so sim can be re-run by reloading page
            sim!(simdata, monitor, [simdata, client])
        end

    end
end


"""
Serve page(s) and supporting css or js files
Files will not be loaded by browser unless they appear in this list!
Paths are relative to the location of this script (or invocation location?)
"""
httph = HttpHandler() do req::Request, res::Response

    # Handle case where / means index.html
    if req.resource == "/"
        println("serving ", req.resource)
        return Response(readall("index.html"))
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
            return Response(open(readall, file))
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
