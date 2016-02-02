using HttpServer
using WebSockets
import JSON

"""
Create a JSON message with waypoint positions
"""
function waypoints()
    wp, _ = readdlm("course1.txt", header=true)
    wp = wp'

    msg = Dict{AbstractString, Any}()
    msg["type"] = "waypoints"
    msg["data"] = [Dict(:x => wp[1,i], :y => wp[2,i]) for i in 1:size(wp, 2)]
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
            write(client, waypoints())
        end

        # if startswith(msg, "getnumber")
        #     write(connections[client.id], waypoints())
        # end

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

    # Not every request is a file, so this is not a general solution.
    # println(req.resource, typeof(req.resource))
    # if isfile("$req.resource")
    #     println("hi")
    #     println("serving /", req.resource)
    #     return Response(open(readall, req.resource))
    #     println("bye")
    # end

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
