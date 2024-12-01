# courtesy of Kevin Tracy
import MeshCat as mc
using ColorTypes
using GeometryBasics: HyperRectangle, Cylinder, Vec, Point, Mesh
using CoordinateTransformations
using Rotations

function rotx(θ)
    s, c = sincos(θ)
    return [1 0 0; 0 c -s; 0 s c]
end

# meshcat animation for quadrotor

function mat_from_vec(X)
    # convert a vector of vectors to a matrix 
    Xm = hcat(X...)
    return Xm 
end

function visualize_quad_state(X)
    # visualize the state history of the quadrotor 
    X_m = mat_from_vec(X)
    display(plot(X_m[1:7,:]',label=["x" "y" "z" "qw" "qx" "qy" "qz"],
    linestyle=[:solid :solid :solid :dash :dash :dash :dash], linewidth=[2 2 2 2 2 2 2],
                 title="State History", xlabel="time (s)", ylabel="x"))

    plot2= (plot(X_m[8:11,:]', label= ["velocity x" "velocity y" "velocity z"], title= "Linear Velocity"))
    display(plot2)
end

function visualize_quad_xy(Xreal, Xref=nothing)
    # visualize the xy position of the quadrotor
    if Xref != nothing
        X_m = mat_from_vec(Xref)
        plot(X_m[2,:],X_m[1,:],label="ref",
        linestyle=:solid, linewidth=2,
                     title="State History", xlabel="y", ylabel="x")
        X_m = mat_from_vec(Xreal)   
        display(plot!(X_m[2,:],X_m[1,:],label="real", linestyle=:dash, linewidth=2,
                    title="State History", xlabel="y", ylabel="x", aspect_ratio=:equal))
    else
        X_m = mat_from_vec(Xreal)   
        display(plot(X_m[2,:],X_m[1,:],label="real", linestyle=:dash, linewidth=2,
                    title="State History", xlabel="y", ylabel="x", aspect_ratio=:equal))
    end
end

function visualize_controls(U)
    # visualize the state history of the quadrotor 
    U_m = mat_from_vec(U)
    display(plot(U_m', label=["rotor 1" "rotor 2" "rotor 3" "rotor 4"],
    linestyle=[:solid :solid :solid :solid :solid], linewidth=[2 2 2 2 2],
                 title="Control History", xlabel="time (s)", ylabel="Thrust"))
end






######### other visualization stuff
# function vis_traj!(vis, name, X; R = 0.1, color = mc.RGBA(0.5, 0.7, 1.0, 1.0))
#     # visualize a trajectory expressed with X::Vector{Vector}
#     for i = 1:(length(X)-1)
#         a = X[i][1:3]
#         b = X[i+1][1:3]
#         cyl = mc.Cylinder(mc.Point(a...), mc.Point(b...), R)
#         mc.setobject!(vis[name]["p"*string(i)], cyl, mc.MeshPhongMaterial(color=color))
#     end
#     for i = 1:length(X)
#         a = X[i][1:3]
#         sph = mc.HyperSphere(mc.Point(a...), R)
#         mc.setobject!(vis[name]["s"*string(i)], sph, mc.MeshPhongMaterial(color=color))
#     end
# end

# # function animate_hexrotor(Xsim, Xref, dt)

# #     # animate quadrotor, show Xref with vis_traj!, and track Xref with the green sphere
# #     vis = mc.Visualizer()
# #     robot_obj = mc.MeshFileGeometry(joinpath(@__DIR__,"hexrotor_assembly_notilt.obj")) 
    

# #     mc.setobject!(vis[:drone][:base], robot_obj, mc.MeshPhongMaterial(color=mc.RGBA(0.6, 0.6, 1.0, 1.0)))
# #     mc.settransform!(vis[:drone][:base], mc.Translation([0,0,0]) ∘ mc.LinearMap(0.002 * I))

# #     vis_traj!(vis, :traj, Xref; R = 0.01, color = mc.RGBA(0.0, 0.0, 1.0, 1.0))
# #     target = mc.HyperSphere(mc.Point(0,0,0.0),0.1)
# #     mc.setobject!(vis[:target], target, mc.MeshPhongMaterial(color = mc.RGBA(0.0,1.0,0.0,0.4)))


# #     anim = mc.Animation(floor(Int,1/dt))
# #     for k = 1:length(Xsim)
# #         mc.atframe(anim, k) do
# #             r = Xsim[k][1:3]
# #             p = Xsim[k][7:9]
# #             mc.settransform!(vis[:drone], mc.compose(mc.Translation(r),mc.LinearMap((dcm_from_mrp(p)))))
# #             mc.settransform!(vis[:target], mc.Translation(Xref[k][1:3]))
# #         end
# #     end
# #     mc.setanimation!(vis, anim)

# #     return (mc.render(vis))
# # end

# # function animate_quadrotor(Xsim, dt)
# #     vis = mc.Visualizer()
# #     #robot_obj = mc.MeshFileGeometry(joinpath(@__DIR__,"hexrotor_assembly_notilt.obj"))
# #     robot_obj = mc.MeshFileGeometry(joinpath(@__DIR__,"Crazyflie_v2.obj"))
# #     #robot_obj = mc.MeshFileGeometry(joinpath(@__DIR__,"quadrotor.obj"))

# #     #mc.setobject!(vis[:drone], robot_obj)

# #     mc.setobject!(vis[:drone][:base], robot_obj, mc.MeshPhongMaterial(color=mc.RGBA(0.6, 0.6, 1.0, 1.0)))
# #     mc.settransform!(vis[:drone][:base], mc.Translation([0,0,0]) ∘ mc.LinearMap(0.001 * I))
# #     mc.setobject!(vis[:drone], robot_obj, mc.MeshPhongMaterial(color=mc.RGBA(0.6, 0.6, 1.0, 1.0)))
# #     mc.settransform!(vis[:drone], mc.Translation([0,0,0]) ∘ mc.LinearMap(0.1 * I))

# #     # vis_traj!(vis, :traj, Xref[1:85]; R = 0.01, color = mc.RGBA(1.0, 0.0, 0.0, 1.0))
# #     # target = mc.HyperSphere(mc.Point(0,0,0.0),0.1)
# #     # mc.setobject!(vis[:target], target, mc.MeshPhongMaterial(color = mc.RGBA(0.0,1.0,0.0,0.4)))

# #     # mc.settransform!(vis[:drone], mc.Translation([0, 0, 0]) ∘ mc.LinearMap(0.01 * I))

# #     # fps = 30  # You can adjust this to your needs
# #     # anim = mc.Animation(fps=fps)

# #     anim = mc.Animation(floor(Int, 1/dt))
# #     for k = 1:length(Xsim)
# #         mc.atframe(anim, k) do
# #             r = Xsim[k][1:3]
# #             p = Xsim[k][7:9]
# #             mc.settransform!(vis[:drone], mc.compose(mc.Translation(r),mc.LinearMap((dcm_from_mrp(p)))))
# #             #mc.settransform!(vis[:target], mc.Translation(Xref[k][1:3]))
# #         end
# #     end
# #     #mc.setanimation!(vis, anim)
# #     mc.setanimation!(vis, anim)
  
    


# #     return (mc.render(vis))
# # end

# # function animate_quadrotor(Xsim, dt)
# #     vis = mc.Visualizer()
# #     robot_obj = mc.MeshFileGeometry(joinpath(@__DIR__,"Crazyflie_v2.obj"))

# #     # Apply material to drone
# #     mc.setobject!(vis[:drone], robot_obj, mc.MeshPhongMaterial(color=mc.RGBA(0.6, 0.6, 1.0, 1.0)))

# #     # Correct orientation (rotate 180 degrees around X-axis)
# #     mc.settransform!(vis[:drone], mc.Translation([0, 0, 0]) ∘ mc.RotationZ(π) ∘ mc.LinearMap(0.01 * I))

# #     anim = mc.Animation(floor(Int, 1/dt))
# #     for k = 1:length(Xsim)
# #         mc.atframe(anim, k) do
# #             r = Xsim[k][1:3]
# #             p = Xsim[k][7:9]
# #             mc.settransform!(vis[:drone], mc.compose(mc.Translation(r), mc.LinearMap(1.5*(dcm_from_mrp(p)))))
# #         end
# #     end
# #     mc.setanimation!(vis, anim)

# #     # Open MeshCat visualization in browser
# #     return mc.render(vis, open_browser=true)
# # end

# # function animate_quadrotor(Xsim, dt)
# #     vis = mc.Visualizer()
# #     robot_obj = mc.MeshFileGeometry(joinpath(@__DIR__,"Crazyflie_v2.obj"))
# #     #mc.setobject!(vis[:vic], robot_obj)

# #     mc.setobject!(vis[:drone][:base], robot_obj, mc.MeshPhongMaterial(color=mc.RGBA(0.6, 0.6, 1.0, 1.0)))
# #     mc.settransform!(vis[:drone][:base], mc.Translation([0,0,0]) ∘ mc.LinearMap(0.1 * I))

# #     #mc.setobject!(vis[:drone], robot_obj, mc.MeshPhongMaterial(color=mc.RGBA(0.6, 0.6, 1.0, 1.0)))
# #     #mc.settransform!(vis[:drone], mc.Translation([0,0,0]) ∘ mc.LinearMap(0.1 * I))

# #     #fps = 30  # Frame rate (frames per second)
# #     fps= floor(Int, 1/dt)
# #     num_frames = length(Xsim)
    
# #     # Loop over each frame and update
# #     for k = 1:num_frames
# #         r = Xsim[k][1:3]  # Position of the quadrotor
# #         p = Xsim[k][7:9]  # Orientation (MRP or other representation)
        
# #         # Set the transformation for the quadrotor (position + orientation)
# #         mc.settransform!(vis[:vic], mc.compose(mc.Translation(r), mc.LinearMap(1.5 * (dcm_from_mrp(p)))))

# #         # Introduce a delay to control the speed of the animation (fps)
# #         sleep(dt)  # dt should match your desired time step for the animation
# #     end

# #     return mc.render(vis)
# # end

# function animate_quadrotor(Xsim, dt)
#     vis = mc.Visualizer()
#     #robot_obj = mc.MeshFileGeometry(joinpath(@__DIR__,"hexrotor_assembly_notilt.obj"))
#     #robot_obj = mc.MeshFileGeometry(joinpath(@__DIR__,"quadrotor.obj"))
#     robot_obj = mc.MeshFileGeometry(joinpath(@__DIR__,"Crazyflie_v2.obj"))

#     #mc.setobject!(vis[:drone], robot_obj)

#     mc.setobject!(vis[:drone][:base], robot_obj, mc.MeshPhongMaterial(color=mc.RGBA(0.3, 0.5, 1.0, 1.0)))
#     #mc.settransform!(vis[:drone][:base], mc.Translation([0,0,0]) ∘ mc.LinearMap(0.001 * I))
#     mc.settransform!(vis[:drone][:base], mc.Translation([0,0,0]) ∘ mc.LinearMap(0.1 * I))


#     # vis_traj!(vis, :traj, Xref[1:85]; R = 0.01, color = mc.RGBA(1.0, 0.0, 0.0, 1.0))
#     # target = mc.HyperSphere(mc.Point(0,0,0.0),0.1)
#     # mc.setobject!(vis[:target], target, mc.MeshPhongMaterial(color = mc.RGBA(0.0,1.0,0.0,0.4)))


#     #     anim = mc.Animation(floor(Int,1/dt))
#     #     for k = 1:length(Xsim)
#     #         mc.atframe(anim, k) do
#     #             r = Xsim[k][1:3,:]
#     #             p = Xsim[k][7:9,:]
#     #             mc.settransform!(vis[:drone], mc.compose(mc.Translation(r),mc.LinearMap(1.5*(dcm_from_mrp(p)))))
#     #             #mc.settransform!(vis[:target], mc.Translation(Xref[k][1:3]))
#     #         end
#     #     end
#     #     mc.setanimation!(vis, anim)

#     #     return (mc.render(vis))
#     # end
#     fps= floor(Int, 1/dt)
#     anim = mc.Animation(vis; fps)
#     for k = 1:length(Xsim)
#         mc.atframe(anim, k) do
#             r = Xsim[k][1:3]
#             p = Xsim[k][7:9]
#             mc.settransform!(vis[:drone], mc.compose(mc.Translation(r),mc.LinearMap(1.5*(dcm_from_mrp(p)))))
#             #mc.settransform!(vis[:target], mc.Translation(Xref[k][1:3]))
#         end
#     end
#     mc.setanimation!(vis, anim)

#     return (mc.render(vis))
# end




# # to do
#     # change all units to kg and m





function rotY(theta)
    [cos(theta) 0 sin(theta);
     0 1 0;
     -sin(theta) 0 cos(theta)]
end
        

function update_quad_pose!(vis, name, x)
    px,pz,theta,_,_,_ = x 
    r = [px,0,pz]
    mc.settransform!(vis[name], mc.compose(mc.Translation(r),mc.LinearMap(1.5*rotY(theta))))
end
function vis_traj!(vis, name, X; R = 0.1, color = mc.RGBA(1.0, 0.0, 0.0, 1.0))
    # visualize a trajectory expressed with X::Vector{Vector}
    for i = 1:(length(X)-1)
        a = X[i][1:3]
        b = X[i+1][1:3]
        cyl = mc.Cylinder(mc.Point(a...), mc.Point(b...), R)
        mc.setobject!(vis[name]["p"*string(i)], cyl, mc.MeshPhongMaterial(color=color))
    end
    for i = 1:length(X)
        a = X[i][1:3]
        sph = mc.HyperSphere(mc.Point(a...), R)
        mc.setobject!(vis[name]["s"*string(i)], sph, mc.MeshPhongMaterial(color=color))
    end
end
function convert_2d_to_3d(X)
    [[x[1],0,x[2]] for x in X]
end
function animate_Crazyflie(x1, dt, Xref)
    # animate quadrotor, show Xref with vis_traj!, and track Xref with the green sphere
    vis = mc.Visualizer()
    robot_obj = mc.MeshFileGeometry(joinpath(@__DIR__,"Crazyflie_v2.obj"))
    
    c1 = mc.RGBA(1.0, 0.0, 0.0, 1.0)
   
    X1 = convert_2d_to_3d(x1)
  
    vis_traj!(vis, Xref, X1; R = 0.01, color = c1)
    
    mc.setobject!(vis[:vic1], robot_obj,mc.MeshPhongMaterial(color=c1))
    
    anim = mc.Animation(floor(Int,1/dt))
    for k = 1:length(X1)
        mc.atframe(anim, k) do
            update_quad_pose!(vis, :vic1, x1[k])
          
        end
    end
    mc.setanimation!(vis, anim)

    return (mc.render(vis))
end

