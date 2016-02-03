using HttpServer
using WebSockets
import JSON

module_dir = "$(pwd())/../.."
module_dir in LOAD_PATH || push!(LOAD_PATH, module_dir)

include("../ekfslam-sim.jl")
scene, vehicle, state = ekfsim_setup(10, "../course1.txt")

function monitor(scene::Scene, vehicle::Vehicle, state::SlamState, client::WebSockets.WebSocket)
    # println(scene.nsteps)

    # TODO: make a Dict to store true and slam track poses
    # msg = Dict{AbstractString, Any}("type" => "test", 
    #                                 "data" => scene.true_track[:, scene.nsteps], 
    #                                 "timestamp" => time())
    n = scene.nsteps
    tt, st = scene.true_track, scene.slam_track
    msg = Dict{AbstractString, Any}()
    msg["type"] = "tracks"
    msg["data"] = Dict("ideal" => Dict(
                            "x"     => tt[1, n],
                            "y"     => tt[2, n],
                            "phi"   => tt[3, n]),
                       "slam" => Dict(
                            "x"     => st[1, n],
                            "y"     => st[2, n],
                            "phi"   => st[3, n])
                       )
    msg["timestamp"] = time()
    write(client, JSON.json(msg))
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
            write(client, xypoints_json(scene.waypoints, "waypoints"))
            write(client, xypoints_json(scene.landmarks, "landmarks"))
        end

        if haskey(msg, "text") && msg["text"] == "start"
            println("Answering request: start")
            sim(scene, vehicle, state, monitor, [scene, vehicle, state, client])
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
