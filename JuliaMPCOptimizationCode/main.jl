import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate()

#Pkg.add("PyCall")

using Libdl
using LinearAlgebra
using BlockDiagonals
import ForwardDiff as FD
#using Plots
#using CSV
import ECOS  
#using ProgressMeter
#import MeshCat as mc 
using Test, Distributions
using Random
using Rotations
#using DelimitedFiles


include("CrazyflieDynamics_withMagnet.jl")
include("convex_mpc.jl")


# optimal trajectory X_sim that the online MPC will reference


# Crazyflie model parameters
model = (
    mass = 0.032499 + 0.001386,  # mass
    J = [(1.66e-5+1.089e-6) 0.83e-6 0.72e-6; 0.83e-6 (1.66e-5+1.041e-6) 1.8e-6; 0.72e-6 1.8e-6 (2.93e-5+0.134e-6)],  # inertia matrix updated with rigid body mass
    g = 9.81,  # gravity
    thrustToTorque = 0.0008,  # thrust to torque ratio
    ℓ = 0.046/1.414213562,  # arm length
    PWM = 65535,  # PWM scale
    kt = 2.245365e-6*65535, # thrust coefficient
    km = 2.245365e-6*65535*0.0008, # moment coefficient
    freq =  100.0, #50.0, # control frequency
    dt = 0.01 # 50 Hz
)

# system setup
nx1 = 13        # number of states (quaternion)
nu = 4          # number of controls
horizonLength = 20  # horizon length

# Hovering state and control input
rg = [0.0; 0; 0.0]
qg = [1.0; 0; 0; 0]
vg = zeros(3)
ωg = zeros(3)
X̄ = [rg; qg; vg; ωg]
magnet_force_z = 2.00
Ū = ((model.mass * model.g + magnet_force_z) / model.kt / 4) * ones(4)
#Ū = (model.mass*model.g/model.kt/4)*ones(4)  # m = 30g and max thrust = 60g

# linearized dynamics
A= [1.0 0.0 0.0 1.6945884581193607e-19 0.0 0.17208291620086372 3.3809587837692277e-19 0.049999999877375 0.0 0.0 0.0 0.0014340243022600218 4.226198479711535e-21; 
    0.0 1.0 0.0 1.6904793918846138e-19 -0.17208291620086372 0.0 -3.3891769162387214e-19 0.0 0.049999999877375 0.0 -0.0014340243022600218 0.0 -4.236471145298402e-21; 
    0.0 0.0 1.0 9.988230795007253e-37 -3.3809587837692277e-19 3.3891769162387214e-19 0.0 0.0 0.0 0.049999999959125 -4.226198479711535e-21 4.236471145298402e-21 0.0; 
    0.0 0.0 0.0 0.0 -5.894179712113206e-18 5.9085067574193146e-18 7.256258862015276e-21 0.0 0.0 0.0 -1.4735449280283018e-19 1.4771266893548287e-19 1.8140647155038193e-22; 
    0.0 0.0 0.0 -5.894179712113206e-18 1.0 -7.256258862015276e-21 5.9085067574193146e-18 0.0 0.0 0.0 0.025 1.5857079868473843e-20 1.168485769376603e-19; 
    0.0 0.0 0.0 5.9085067574193146e-18 7.256258862015276e-21 1.0 5.894179712113206e-18 0.0 0.0 0.0 -1.671269491788296e-21 0.025 1.1743763838008154e-19; 
    0.0 0.0 0.0 7.256258862015276e-21 -5.9085067574193146e-18 -5.894179712113206e-18 1.0 0.0 0.0 0.0 -5.493068343024437e-20 -5.55114457462995e-20 0.025; 
    0.0 0.0 0.0 1.3556707664954884e-17 0.0 6.883316648034549 2.7047670270153818e-17 0.9999999959125 0.0 0.0 -3.1881388306782485e-21 0.08604145817077076 4.556620219549785e-19; 
    0.0 0.0 0.0 1.3523835135076909e-17 -6.883316648034549 0.0 -2.711341532990977e-17 0.0 0.9999999959125 0.0 -0.08604145817077076 -2.759949505351604e-20 -4.552647077801999e-19; 
    0.0 0.0 0.0 1.1985876954008702e-34 -2.7047670270153818e-17 2.711341532990977e-17 0.0 0.0 0.0 0.9999999991825 -5.071438175653842e-19 5.083765374358083e-19 5.394199898609928e-38; 
    0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0 1.910105843078876e-18 8.113322475099922e-18; 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 -2.0780859787661077e-19 1.0 8.19833689349658e-18; 
    0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 -6.831752542100091e-19 -7.671937774427341e-19 1.0]

B= [-0.004678438108942158 0.005114649104284945 0.0046929270307515805 -0.0051291380260943655; 
    0.004644105073545807 0.005085513615858357 -0.004649302620515573 -0.005080316068888594; 
    0.005428286678286853 0.005428286678286853 0.005428286678286853 0.005428286678286853; 
    0.0 0.0 0.0 0.0; 
    -0.1619256054633058 -0.17731615850608243 0.162106827878965 0.17713493609042327; 
    -0.16312269240597968 0.17833202303309714 0.1636278763756244 -0.17883720700274186; 
    0.011436851417640947 -0.004068594472300589 -0.01647149864736287 0.009103241702022513; 
    -0.37427504825641783 0.4091719278410485 0.3754341619997502 -0.4103310415843808; 
    0.37152840542807797 0.4068410887697797 -0.3719442091851492 -0.4064252850127085; 
    0.21713146713147413 0.21713146713147413 0.21713146713147413 0.21713146713147413; 
    -12.954048437064463 -14.185292680486596 12.968546230317198 14.17079488723386; 
    -13.049815392478372 14.266561842647771 13.090230110049951 -14.306976560219349; 
    0.9149481134112759 -0.32548755778404714 -1.3177198917890294 0.7282593361618009]

# MPC matrices
Q= [399.99999999999994 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0; 
    0.0 399.99999999999994 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0; 
    0.0 0.0 399.99999999999994 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0; 
    0.0 0.0 0.0 399.99999999999994 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0; 
    0.0 0.0 0.0 0.0 2.8727377190462513 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0; 
    0.0 0.0 0.0 0.0 0.0 1111.111111111111 0.0 0.0 0.0 0.0 0.0 0.0 0.0; 
    0.0 0.0 0.0 0.0 0.0 0.0 1111.111111111111 0.0 0.0 0.0 0.0 0.0 0.0; 
    0.0 0.0 0.0 0.0 0.0 0.0 0.0 4.0 0.0 0.0 0.0 0.0 0.0; 
    0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 4.0 0.0 0.0 0.0 0.0; 
    0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 4.0 0.0 0.0 0.0; 
    0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 2.0408163265306127 0.0 0.0; 
    0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 2.0408163265306127 0.0; 
    0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 24.999999999999996]


R= [3600.0 0.0 0.0 0.0; 
    0.0 3600.0 0.0 0.0; 
    0.0 0.0 3600.0 0.0; 
    0.0 0.0 0.0 3600.0]

Qf= [2674.1180654371765 -15.328472379830192 -4.684637149296505e-14 1.1203562442961511e-14 73.97012154499713 3315.490690447928 7.449609217139722 340.09469427145825 -4.536309765748227 -6.78343925059502e-14 1.8306565897440221 51.11720515330605 -1.2266646603865452; 
    -15.328472379830192 2579.2582847512103 1.8325124610922603e-13 1.1013119364251107e-14 -3172.6655441912926 -83.47854543208175 -2.967391526431312 -4.764040569378466 311.3192553042376 1.488399685624136e-13 -51.02676357371657 -2.06588561289009 0.5098255027916961; 
    -4.684637149296505e-14 1.8325124610922603e-13 6900.298759620415 -1.4030365967938755e-30 -6.853112633267155e-14 -3.955439266447825e-13 -1.644661886631153e-12 -1.6615894750505463e-14 3.1449305557318573e-14 2763.375215777795 -2.261699409275491e-14 -5.70727211296443e-14 -7.396342530756325e-13; 
    1.1203562442961511e-14 1.1013119364251107e-14 -1.4030365967938755e-30 399.99999999999994 -5.118514384479866e-14 5.92943477361213e-14 4.774212091139839e-15 3.302613657063801e-15 3.103214286016566e-15 1.819889363427595e-30 -1.0640869052052437e-15 1.2983608573953299e-15 1.328719937470555e-15; 
    73.97012154499713 -3172.6655441912926 -6.853112633267155e-14 -5.118514384479866e-14 12652.94675566642 536.1658121605966 542.3983040648875 25.345778688820836 -830.5112451326106 -2.792259047460417e-13 265.2856605934963 23.904347833318788 148.07077405339265; 
    3315.490690447928 -83.47854543208175 -3.955439266447825e-13 5.92943477361213e-14 536.1658121605966 15461.266266811206 1355.8267442535678 911.6959539672628 -27.092444077953253 9.07410846712959e-14 23.77118029393138 307.034277171947 370.1099415203911; 
    7.449609217139722 -2.967391526431312 -1.644661886631153e-12 4.774212091139839e-15 542.3983040648875 1355.8267442535678 22676.134857155503 12.178451016864095 -4.870267313015258 -1.3959778044717507e-12 122.44094019678835 306.0872359140426 5001.380109892329; 
    340.09469427145825 -4.764040569378466 -1.6615894750505463e-14 3.302613657063801e-15 25.345778688820836 911.6959539672628 12.178451016864095 83.10898769583034 -1.4637449404223826 -4.450736977187253e-15 0.6972687962158148 15.569955008815878 1.29628350678859; 
    -4.536309765748227 311.3192553042376 3.1449305557318573e-14 3.103214286016566e-15 -830.5112451326106 -27.092444077953253 -4.870267313015258 -1.4637449404223826 71.55190382630391 3.985784705772978e-14 -14.865102594614882 -0.7445299497506009 -0.5138711145543196; 
    -6.78343925059502e-14 1.488399685624136e-13 2763.375215777795 1.819889363427595e-30 -2.792259047460417e-13 9.07410846712959e-14 -1.3959778044717507e-12 -4.450736977187253e-15 3.985784705772978e-14 2316.429925057803 -1.968479638484947e-14 -5.467543683182818e-14 -9.49685974578554e-13; 
    1.8306565897440221 -51.02676357371657 -2.261699409275491e-14 -1.0640869052052437e-15 265.2856605934963 23.77118029393138 122.44094019678835 0.6972687962158148 -14.865102594614882 -1.968479638484947e-14 11.769212490308902 4.3428442779953755 60.92251783482003; 
    51.11720515330605 -2.06588561289009 -5.70727211296443e-14 1.2983608573953299e-15 23.904347833318788 307.034277171947 306.0872359140426 15.569955008815878 -0.7445299497506009 -5.467543683182818e-14 4.3428442779953755 19.961512669804023 152.29717431370622; 
    -1.2266646603865452 0.5098255027916961 -7.396342530756325e-13 1.328719937470555e-15 148.07077405339265 370.1099415203911 5001.380109892329 1.29628350678859 -0.5138711145543196 -9.49685974578554e-13 60.92251783482003 152.29717431370622 2501.7514044837385]

# constraints
x_min = -1000. * ones(nx1)  # state constraints
x_max = 1000. * ones(nx1)  # state constraints

u_min = (-1.686 * ones(nu)).- Ū  # force constraints
u_max = (5.6 * ones(nu)) .- Ū ; # force constraints


# everything above compiled once. 

# need to find the part of the trajectory that is the closest to the measured state from the drone
# function find_closest_point(ref_traj::Vector{Vector{Float64}}, current_state::Vector{Float64})
#     closest_point = ref_traj[1]
#     min_distance = norm(current_state[1:2] .- closest_point[1:2])  # Calculate initial distance between current_state and the first point
    
#     for point in trajectory
#         distance = norm(current_state[1:2] .- point[1:2])  # Calculate the Euclidean distance for x and y positions only
#         if distance < min_distance
#             min_distance = distance
#             closest_point = point
#         end
#     end
    
#     return closest_point, closest_point_index
# end

function find_closest_point_index(ref_traj::Vector{Vector{Float64}}, current_state::Vector{Float64})
    closest_point_index = 1
    min_distance = norm(current_state[1:2] .- ref_traj[1][1:2])  # Calculate initial distance for x and y positions

    for i in 2:length(ref_traj)
        distance = norm(current_state[1:2] .- ref_traj[i][1:2])  # Calculate Euclidean distance for x and y
        if distance < min_distance
            min_distance = distance
            closest_point_index = i
        end
    end
    
    closest_point = ref_traj[closest_point_index]
    return closest_point_index
end


# go on same trajectory but starting at the current state then the state on the trajectory closest to it
function create_traj(ref_traj::Vector{Vector{Float64}}, current_state::Vector{Float64}, goal_state::Vector{Float64}, closest_point_idx::Int, horizonLength::Int)
    # Create the new trajectory
    new_traj = [current_state]  # Start with the current state
    push!(new_traj, ref_traj[closest_point_idx])  # Add the closest point
    new_traj = vcat(new_traj, ref_traj[closest_point_idx+1:end], goal_state)  # Append the remaining trajectory

    # pad the end of the trajectory with terminal states
    new_traj = [new_traj...,[new_traj[length(new_traj)] for i = 1:(length(new_traj)+horizonLength)]...]  # Xref for MPC, padded with xgoals

    return new_traj
end


# MPC calculation
# function run_mpc(current_state::Vector{Float64}, xref::Vector{Vector{Float64}}, goal_state::Vector{Float64}) #) # state is from the onboard observer and will be the initial condition for MPC, tracking the optimal reference traj

#     horizonLength= 10;

#     # get closes point index
#     closest_ref_index= find_closest_point_index(xref, current_state)

#     # get the new traj
#     Xref= create_traj(xref, current_state, goal_state, closest_ref_index, horizonLength)

#     # set up simulation
#     NSIM= (length(new_traj)/2 ) - horizonLength 

#     X_sim = [zeros(nx1) for i = 1:NSIM]
#     X_sim[1] = 1*Xref[1] # + randn(n_states) * 0.001
#     deltaX_sim = [zeros(nx1) for i = 1:NSIM]
#     deltaX_sim[1] = X_sim[1] - X̄ 
    

#     # simulation
#     u_current = [zeros(nu) for i in 1:NSIM-1]
#     for i in 1:(NSIM-1)
        
#         xref_horizon = xref[i: (i+horizonLength-1)]  
#         # call convex mpc controller with state estimate 
#         u_current[i] = convex_mpc(A, B, xref_horizon, deltaX_sim[i], u_min, u_max, x_min, x_max, horizonLength, Q, R, Qf, Ū)  #+ randn(n_inputs) * 0.01
#         # sim forward
#         X_sim[i+1] = quad_dynamics_rk4(X_sim[i], u_current[i], model.dt)
#         # update for next MPC call
#         deltaX_sim[i+1]= X_sim[i+1] - X̄

#     end

    #return X_sim

    # MPC calculation
    function run_mpc(current_state::Vector{Float64}, xref_matrix::Matrix{Float64}, goal_state::Vector{Float64}) #) # state is from the onboard observer and will be the initial condition for MPC, tracking the optimal reference traj

        horizonLength= 10;

        #convert to vector of vectors
        # Convert the matrix to a vector of vectors in Julia
        xref = [Vector{Float64}(xref_matrix[:, i]) for i in 1:size(xref_matrix, 2)]


        # get closes point index
        closest_ref_index= find_closest_point_index(xref, current_state)

        # get the new traj
        Xref= create_traj(xref, current_state, goal_state, closest_ref_index, horizonLength)

        xref_horizon = xref[1: (horizonLength)]  

        # set up simulation
        #NSIM= (length(new_traj)/2 ) - horizonLength 

        X_sim = 1*Xref[1] # + randn(n_states) * 0.001
        deltaX_sim = X_sim[1] .- X̄ 
    
        # call convex mpc controller with state estimate 
        u_current = convex_mpc(A, B, xref_horizon, deltaX_sim, u_min, u_max, x_min, x_max, horizonLength, Q, R, Qf, Ū)  #+ randn(n_inputs) * 0.01
        # sim forward
        X_sim = quad_dynamics_rk4(X_sim, u_current, model.dt)
        # update for next MPC call
        #deltaX_sim[i+1]= X_sim[i+1] - X̄

        return X_sim

        
end