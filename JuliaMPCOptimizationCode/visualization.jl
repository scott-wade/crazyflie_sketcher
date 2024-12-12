
import MeshCat as mc
using ColorTypes
using GeometryBasics: HyperRectangle, Cylinder, Vec, Point, Mesh
using CoordinateTransformations
using VideoIO

################## PLOTTING #################
function rotx(θ)
    s, c = sincos(θ)
    return [1 0 0; 0 c -s; 0 s c]
end

function mat_from_vec(X)
    # convert a vector of vectors to a matrix 
    Xm = hcat(X...)
    return Xm 
end

function quaternion_to_euler(q)
    # Assuming q is a quaternion in the form (w, x, y, z)
    w, x, y, z = q
    
    # Compute the Euler angles (Roll, Pitch, Yaw)
    roll = atan(2*(w*x + y*z), 1 - 2*(x^2 + y^2))
    pitch = asin(2*(w*y - z*x))
    yaw = atan(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    
    return roll, pitch, yaw
end


function visualize_quad_state(X, Xref, filename::String, filename2::String, filename3::String, filename4::String)
    #convert quaternions to euler angles for visualization
    quaternions = [state[4:7] for state in X]
    euler_angles = []
    for q in quaternions
        roll, pitch, yaw = quaternion_to_euler(q)
        push!(euler_angles, [roll, pitch, yaw])
    end

    # visualize the state history of the quadrotor 
    X_m = mat_from_vec(X)
    Xref_m= mat_from_vec(Xref)

    # Define base colors for x, y, z components
    color_x = :blue        # Color for x and x_ref
    color_y = :green       # Color for y and y_ref
    color_z = :red         # Color for z and z_ref

    # Define lighter versions of those colors for the ref
    lighter_x = RGBA(0.6, 0.6, 1.0, 0.6)  # Lighter blue for x_ref
    lighter_y = RGBA(0.6, 1.0, 0.6, 0.6)  # Lighter green for y_ref
    lighter_z = RGBA(1.0, 0.6, 0.6, 0.6)  # Lighter red for z_ref

    
    plot(X_m[1,:], label="x", linestyle=:solid, linewidth=2, color=color_x, xlabel="Trajectory Progression", ylabel="Position (m)", dpi=300)
    plot!(Xref_m[1,:], label="x reference", linestyle=:dot, linewidth=3, color=lighter_x)

    plot!(X_m[2,:], label="y", linestyle=:solid, linewidth=2, color=color_y)  # y-component
    plot!(Xref_m[2,:], label="y reference", linestyle=:dot, linewidth=3, color=lighter_y)  # y_ref

    plot!(X_m[3,:], label="z", linestyle=:solid, linewidth=2, color=color_z)  # z-component
    display(plot!(Xref_m[3,:], label="z reference", linestyle=:dot, linewidth=3, color=lighter_z))  # z_ref

    # Save the plot as an image
    savefig(filename)


    euler_m= mat_from_vec(euler_angles)
    plot2= (plot(euler_m', label= ["roll (x)" "pitch (y)" "yaw (z)"],linestyle=:solid, linewidth=2,  xlabel= "trajectory progression", ylabel= "angle (rad)", dpi=300))
    savefig(filename2)
    display(plot2)
    #title= "Orientation History",
    # Save the plot as an image
    savefig(filename2)

    plot(X_m[8, :], label="vx", linestyle=:solid, linewidth=2, color=color_x, 
        xlabel="Trajectory Progression", ylabel="velocity (m/s)",  dpi=300)
    plot!(Xref_m[8, :], label="vx reference", linestyle=:dot, linewidth=2, color=lighter_x)

    plot!(X_m[9, :], label="vy", linestyle=:solid, linewidth=2, color=color_y)
    plot!(Xref_m[9, :], label="vy reference", linestyle=:dot, linewidth=2, color=lighter_y)

    plot!(X_m[10, :], label="vz", linestyle=:solid, linewidth=2, color=color_z)
    display(plot!(Xref_m[10, :], label="vz reference", linestyle=:dot, linewidth=2, color=lighter_z))
    #title="Linear Velocity History",

    # Save the plot
    savefig(filename3)

    plot4= (plot(X_m[11:13,:]', label= ["wx" "wy" "wz"], linestyle=:solid, linewidth=2, xlabel= "trajectory progression", ylabel= "angular velocity (rad/s)", dpi=300))
    #title= "Angular Velocity History"
    savefig(filename4)
    display(plot4)
end

function visualize_quad_xy(Xreal,filename::String, Xref=nothing)
    # visualize the xy position of the quadrotor
    if Xref != nothing
        X_m = mat_from_vec(Xref)
        plot(X_m[1,:],X_m[2,:],label="ref",
        linestyle=:solid, linewidth=2,
                     title="State History", xlabel="x", ylabel="y")
        X_m = mat_from_vec(Xreal)   
        display(plot!(X_m[1,:],X_m[2,:],label="real", linestyle=:dash, linewidth=2,
                    title="State History", xlabel="x", ylabel="y", aspect_ratio=:equal, dpi=300))
        
    else
        X_m = mat_from_vec(Xreal)   
        display(plot(X_m[1,:],X_m[2,:],label="real", linestyle=:dash, linewidth=2,
                    title="State History", xlabel="x", ylabel="y", aspect_ratio=:equal, dpi=300))
       
    end
    savefig(filename)
end

function visualize_controls(U, filename::String)
    # visualize the state history of the quadrotor 
    U_m = mat_from_vec(U)
    display(plot(U_m', label=["rotor 1" "rotor 2" "rotor 3" "rotor 4"],
    linestyle=[:solid :solid :solid :solid :solid], linewidth=[2 2 2 2 2],
                  xlabel="trajectory progression", ylabel="Thrust (N)", dpi=300))
    savefig(filename)
    #title="Control History",
end

function error_calculation(Xref, X_sim, filename::String)
    # # X_m_ref = mat_from_vec(Xref)
    

    # # Compute error as the difference between reference and simulated states
    errors = [Xref[i] .- X_sim[i] for i in 1:length(Xref)]
    state_errors = mat_from_vec(errors)
    
    #subplot
    plot_layout = @layout([a; b])
    # position errors
    p1= plot(state_errors[1:3, :]', label=["x" "y" "z"], title= "Position States", titlefont= font(10))
    # velocity errors
    p3= plot(state_errors[8:10, :]', label=["vx" "vy" "vz"], title= "Velocity States", titlefont= font(10))

    # Only set the axis labels for the outermost subplots
    xlabel!(p3, "Progression along Trajectory")
    ylabel!(p1, "Absolute Error")
    ylabel!(p3, "Absolute Error")
   
    display(plot(p1, p3, layout=plot_layout, dpi=300)) #, xlabel="Progression along Trajectory", ylabel="Error", grid=false))
    savefig(filename)

end

    
################## ANIMIATION ######################

function rot3D(yaw, pitch, roll)
    # Rotation about Z-axis (yaw)
    Rz = [cos(yaw) -sin(yaw) 0;
          sin(yaw)  cos(yaw) 0;
          0         0        1]
    # Rotation about Y-axis (pitch)
    Ry = [cos(pitch)  0 sin(pitch);
          0           1 0;
         -sin(pitch)  0 cos(pitch)]
    # Rotation about X-axis (roll)
    Rx = [1  0          0;
          0  cos(roll) -sin(roll);
          0  sin(roll)  cos(roll)]
    return Rz * Ry * Rx  # Combine rotations in order: yaw -> pitch -> roll
end

function update_quad_pose!(vis, name, x)
    px, py, pz, yaw, pitch, roll = x  # Include all state variables
    r = [px, py, pz]                 # Position vector (X, Y, Z)
    rotation = rot3D(yaw, pitch, roll)  # Compute full 3D rotation
    mc.settransform!(vis[name], mc.compose(mc.Translation(r), mc.LinearMap(rotation)))
end


function vis_traj!(vis, name, X; R = 0.1, color = mc.RGBA(0.18, 0.55, 0.34, 1.0))
    for i = 1:(length(X) - 1)
        a = X[i][1:3]
        b = X[i + 1][1:3]
        cyl = mc.Cylinder(mc.Point(a...), mc.Point(b...), R)
        
        # # Print the name to debug
        # println("Creating cylinder with name: ", name * "_p" * string(i))
        
        mc.setobject!(vis[name]["p" * string(i)], cyl, mc.MeshPhongMaterial(color=color))
    end
    
    for i = 1:length(X)
        a = X[i][1:3]
        sph = mc.HyperSphere(mc.Point(a...), R)
        
        # # Print the name to debug
        # println("Creating sphere with name: ", name * "_s" * string(i))
        mc.setobject!(vis[name]["s" * string(i)], sph, mc.MeshPhongMaterial(color=color))
    end
end

function convert_2d_to_3d(X)
    [[x[1],x[2],x[3]] for x in X]
end 

function vis_traj!(vis, name, X; R = 0.1, color = mc.RGBA(0.8, 0.6, 0.9, 0.5))
    # Add cylinders to represent the trajectory as a line
    for i = 1:(length(X) - 1)
        a = X[i][1:3]
        b = X[i + 1][1:3]


        if norm(a .- b) < 1e-6
            println("Skipping identical points at index ", i, ": ", a)
        else
            # Only create a cylinder if points are not identical
            cyl = mc.Cylinder(mc.Point(a...), mc.Point(b...), R)
            mc.setobject!(vis[name]["p" * string(i)], cyl, mc.MeshPhongMaterial(color=color))
            last_valid_point = b
        end

        # cyl = mc.Cylinder(mc.Point(a...), mc.Point(b...), R)
        # mc.setobject!(vis[name]["p" * string(i)], cyl, mc.MeshPhongMaterial(color=color))
    end
    
    # Add spheres to represent waypoints along the trajectory
    for i = 1:length(X)
        a = X[i][1:3]
        sph = mc.HyperSphere(mc.Point(a...), R)
        mc.setobject!(vis[name]["s" * string(i)], sph, mc.MeshPhongMaterial(color=color))
    end
end

function animate_Crazyflie(x1, dt, Xref)
    vis = mc.Visualizer()

    # Load and configure the drone model
    robot_obj = mc.MeshFileGeometry(joinpath(@__DIR__, "Crazyflie_magnet_v2.obj"))
    c1 = mc.RGBA(0.8, 0.6, 0.9, 0.5)  # Color for the trajectory
    X1 = convert_2d_to_3d(x1)  # Assuming you have a function to convert the trajectory data

    # Add the trajectory to the visualizer
    vis_traj!(vis, "trajectory", Xref; R = 0.003, color = c1)

    # Add the drone model to the visualization
    mc.setobject!(vis[:vic1][:base], robot_obj, mc.MeshPhongMaterial(color=mc.RGBA(0.792, 0.922, 0.09, 1.0)))
    mc.settransform!(vis[:vic1][:base], mc.Translation([0, 0, 0]) ∘ mc.LinearMap(0.001 * I))

    # Create and set the animation
    fps = floor(Int, 1/dt)
    anim = mc.Animation(vis; fps)
    for k = 1:length(X1)
        mc.atframe(anim, k) do
            update_quad_pose!(vis, :vic1, x1[k])
        end
    end
    mc.setanimation!(vis, anim)

    println("Rendering visualization")
    return mc.render(vis)
end

