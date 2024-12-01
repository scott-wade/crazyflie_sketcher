# Rotation stuff
T = Diagonal([1; -ones(3)])
H = [zeros(1,3); I]

function skew(ω::Vector)
    return [0    -ω[3]  ω[2];
            ω[3]  0    -ω[1];
           -ω[2]  ω[1]  0]
end

function L(q)
    s = q[1]
    v = q[2:4]
    L = [s    -v';
         v  s*I+skew(v)]
    return L
end

function G(q)
    G = L(q)*H
end

function qtoQ(q)
    return H'*T*L(q)*T*L(q)*H
end

function rptoq(ϕ)
    (1/sqrt(1+ϕ'*ϕ))*[1; ϕ]
end

function qtorp(q)
    q[2:4]/q[1]
end

function E(q)
    E = BlockDiagonal([1.0*I(3), G(q), 1.0*I(6)])
end


# Quadrotor dynamics
function quad_dynamics(model,x,u)

    #println("in dynamics: ", typeof(x))

    r = x[1:3]
    q = x[4:7]/LinearAlgebra.norm(x[4:7]) #normalize q
    v = x[8:10]
    ω = x[11:13]
    Q = qtoQ(q)

    kt= model.kt
    km= model.km
    ℓ= model.ℓ
    J= model.J
    mass= model.mass
    g= model.g
    
    
    ṙ = v
    q̇ = 0.5*L(q)*H*ω
    
    v̇ = [0; 0; -g] + (1/mass)*Q*[zeros(2,4); kt*ones(1,4)]*u 
  
    ω̇ = J\(-skew(ω)*J*ω + [-ℓ*kt -ℓ*kt ℓ*kt ℓ*kt; -ℓ*kt ℓ*kt ℓ*kt -ℓ*kt; -km km -km km]*u)
    
    #x= [ṙ; q̇; v̇; ω̇] 
    #println("in dynamics after equations: ", typeof(x))
    return [ṙ; q̇; v̇; ω̇]  # Ensure x is a concrete array
    
end
#RK4 integration with zero-order hold on u
function quad_dynamics_rk4(x,u, hertz)   
    
  
    #println("in rk4: ", typeof(x))

    f1 = quad_dynamics(model, x, u)
    # println("f1: ", f1)
    # println("x: ", x)
    f2 = quad_dynamics(model, x + 0.5*hertz*f1, u)
    f3 = quad_dynamics(model, x + 0.5*hertz*f2, u)
    f4 = quad_dynamics(model, x + hertz*f3, u)
    xn = x + (hertz/6.0)*(f1 + 2*f2 + 2*f3 + f4)
    xn[4:7] .= xn[4:7]/LinearAlgebra.norm(xn[4:7]) #re-normalize quaternion
    return xn
end