# SLAM.jl
## Simultaneous localization and mapping for mobile autonomous vehicles

This project implements a few SLAM algorithms in two dimensions using rangefinder data for vehicle localization and for environment mapping. It is based on a Matlab implementation by Tim Bailey et al [[OpenSLAM.org](http://openslam.org/bailey-slam.html)]. This is a work-in-progress port to Julia for fun and self-education, and (hopefully) eventual deployment on a real embedded system.

Current status: the extended Kalman Filter algorithm is written, and FastSLAM is ongoing.

A browser-based visualization of the EKF-SLAM simulation is runnable by launching wsserver.jl and navigating to localhost:8000. It depends on the [HttpServer](https://github.com/JuliaWeb/HttpServer.jl), [WebSockets](https://github.com/JuliaWeb/WebSockets.jl), and [JSON](https://github.com/JuliaLang/JSON.jl) packages, and makes use of D3.js for basic 2-D visualization.
