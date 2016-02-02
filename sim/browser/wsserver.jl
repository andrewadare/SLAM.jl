using HttpServer
using WebSockets
import JSON

module_dir = "$(pwd())/../.."
module_dir in LOAD_PATH || push!(LOAD_PATH, module_dir)

include("../ekfslam-sim.jl")

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

        if haskey(msg, "text") && msg["text"] == "get_waypoints"
            println("Answering request: get_waypoints")
            write(client, xypoints_json(scene.waypoints, "waypoints"))
        end

        if haskey(msg, "text") && msg["text"] == "get_landmarks"
            println("Answering request: get_landmarks")
            write(client, xypoints_json(scene.landmarks, "landmarks"))
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
