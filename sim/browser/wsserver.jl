using HttpServer
using WebSockets
import JSON

module_dir = "$(pwd())/../.."
module_dir in LOAD_PATH || push!(LOAD_PATH, module_dir)

include("../ekfslam-sim.jl")

# Global object containing simulation state, updated in real time.
# It is exposed for monitoring, debugging, and flexible visualization.
simdata = ekfsim_setup(10, "../course1.txt")

"""
Create array of Dicts from key strings and first two rows of array.
"""
function pair_array(a::Matrix, key1::AbstractString, key2::AbstractString)
    [Dict(key1 => a[1,i], key2 => a[2,i]) for i in 1:size(a, 2)]
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

    # Send observations
    if simdata.state_updated && simdata.nz > 0
        z = simdata.z[:, 1:simdata.nz]
        send_json("observations", pair_array(z, "range", "bearing"), client)
    end

    return
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
            send_json("waypoints", pair_array(simdata.scene.waypoints, "x", "y"), client)
            send_json("landmarks", pair_array(simdata.scene.landmarks, "x", "y"), client)
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
