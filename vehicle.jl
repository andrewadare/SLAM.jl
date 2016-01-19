type Vehicle{T<:Real}

    ## Intrinsic properties ##
    wheelbase::T     # Front-to-rear wheel separation [m]
    max_gamma::T     # Max steering angle [rad] (-max_g < g < max_g)
    steer_rate::T    # Maximum rate of change in steer angle [rad/s]
    shape::Matrix{T} # Set of x,y points defining a polygon for visualization

    ## (Potentially) time-varying properties ##
    pose::Vector{T}  # [x; y; phi]

    target_speed::T  # Velocity magnitude (control setpoint) [m/s]
    measured_speed::T  # Measured velocity magnitude [m/s]

    target_gamma::T  # Target steering angle [rad]
    measured_gamma::T  # Measured steering angle [rad]

    waypoint_id::Int  # Index of current target waypoint
end
Vehicle() = Vehicle(0., 0., 0., zeros(2,0), zeros(3), 0., 0., 0., 0., 0)


"""
Advance equations of motion for vehicle by dt and update vehicle pose in-place
This uses the ideal/target speed and steering angle
"""
function step_vehicle!(vehicle::Vehicle, dt::AbstractFloat)
    
    x, y, phi = vehicle.pose
    v = vehicle.target_speed
    g = vehicle.target_gamma

    vehicle.pose = [x + v*dt*cos(g + phi); 
                    y + v*dt*sin(g + phi);
                    mpi_to_pi(phi + v*dt*sin(g)/vehicle.wheelbase)]
end


"""
Find the next target waypoint given the current position and steer for it.
d_min is minimum distance to current waypoint before switching to next
This uses the ideal/target steering angle
"""
function steer!(vehicle::Vehicle, waypoints::AbstractArray, d_min::AbstractFloat, dt::AbstractFloat)

    # Get current vehicle steering angle and waypoint index
    g = vehicle.target_gamma
    iwp = vehicle.waypoint_id
    x, y, phi = vehicle.pose

    # Get coords of current waypoint
    cwp = waypoints[:, iwp]
    
    # Determine if current waypoint has been reached
    d2 = (cwp[1] - x)^2 + (cwp[2] - y)^2
    if d2 < d_min^2
        iwp += 1             # Advance to next waypoint
        if iwp > size(waypoints, 2) # Final waypoint reached, flag and return
            iwp = 0
            vehicle.waypoint_id = iwp
            return
        end
        cwp = waypoints[:, iwp] # Coords of next waypoint
    end

    # Change in steering angle required to point towards current waypoint
    dg = mpi_to_pi(atan2(cwp[2] - y, cwp[1] - x) - phi - g)

    # Constrain steering angle by max steering rate
    dgmax = vehicle.steer_rate*dt
    if abs(dg) > dgmax
        dg = sign(dg)*dgmax
    end

    # Limit steering angle to max steering angle
    g += dg
    if abs(g) > vehicle.max_gamma
        g = sign(g)*vehicle.max_gamma
    end

    vehicle.target_gamma = g
    vehicle.waypoint_id = iwp

    return
end

