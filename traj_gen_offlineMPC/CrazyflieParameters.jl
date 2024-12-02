# drone parameters

model = (
    # Quadrotor parameters
    mass = 0.035,  # mass
    J = [1.66e-5 0.83e-6 0.72e-6; 0.83e-6 1.66e-5 1.8e-6; 0.72e-6 1.8e-6 2.93e-5],  # inertia
    g = 9.81,  # gravity
    thrustToTorque = 0.0008,  # thrust to torque ratio
    ℓ = 0.046/1.414213562,  # arm length
    scale = 65535,  # PWM scale
    kt = 2.245365e-6*scale, # thrust coefficient, u is PWM in range [0...1], 0 is no thrust, 1 is max thrust
    km = kt*thrustToTorque, # moment coefficient
    freq = 50.0, #control frequency
    hertz= 1/freq #50 Hz
)


# max thrust = 60 g