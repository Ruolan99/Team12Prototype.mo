model PlaneDynamic

    // States
    Real thr(start=0.0);
    Real alpha(start=0.0);
    Real theta(start=0.05);
    Real gamma(start=0.0);
    Real v(start=0.01);
    Real h(start=0);
    Real x(start=0);
    Real theta_filt(start=0.0);
    Real m_fuel(start=0);  // FIXED: Added start value
    
    // Phase tracker
    discrete Integer phase(start=1);
    
    // Constants
    parameter Real T_gen = 150000;
    parameter Real g = 9.81;
    parameter Real s = 122.6;
    parameter Real Cl0 = 0.6;
    parameter Real Cl_alpha = 4.39;
    parameter Real Cd0 = 0.022;
    parameter Real k = 0.045;
    parameter Real kp_h = 0.00005;       // Altitude loop gain (h → theta)
    parameter Real kp_theta = 2;         // Pitch loop gain (theta → alpha)
    parameter Real kp_alpha = 5;         // Alpha loop gain
    parameter Real kv = 0.5;
    parameter Real kb = 0.5;
    parameter Real theta_rate_limit = 0.05;
    parameter Real m_tank = 20000;
    parameter Real rate = 0.75;
    
    // Variables
    Real rho, Cl, Cd, m; 
    Real L, D, T, W, B, N;
    Real Fl, Ft;
    Real Fv, Fh;
    Real h_t, theta_t, v_t, x_t;
    Real theta_base;                // Base theta from altitude controller
    Real theta_cmd;

    Real alpha_cmd;
    Real thr_cmd;
    Boolean altitude_hold;

algorithm
  // Phase transitions
  when v > 80 and phase == 1 then
    phase := 2;
  elsewhen h > 11000 and phase == 2 then
    phase := 3;
  elsewhen (x_t - x) < 6000 and phase == 3 then
    phase := 4;
  elsewhen h <= 50 and h > 0 and phase == 4 then
    phase := 5;
  elsewhen h <= 0 and phase == 5 then
    phase := 6;
  elsewhen h <= 0 and v < 50 and phase == 6 then 
    phase := 7;
  end when;

equation
  // Fuel and mass
  der(m_fuel) = -rate*thr;
  m = 50000 + max(0, m_tank + m_fuel);
  
  // Aerodynamics
  rho = 1.225 * exp(-h / 8500);
  Cl = Cl0 + Cl_alpha * alpha;
  Cd = Cd0 + k * Cl^2;
  L = 0.5 * rho * v^2 * s * Cl;
  D = 0.5 * rho * v^2 * s * Cd;
  T = thr * T_gen;
  W = m * g;
  B = if phase == 6 or phase == 7 then kb*(m*v^2)/2000 else 0;
  N = if phase == 1 or phase == 6 or phase == 7 then W*cos(gamma) - L + T * sin(alpha) else 0;

  // Hierarchical control with altitude hold
  theta_base = if altitude_hold then max(-0.2, min(0.4, theta_t + kp_h * (h_t - h))) else theta_t;
  
  // Middle loop: Pitch tracking
  theta_cmd = theta_base + kp_theta * (theta_base - theta);
  
  // Rate limiting on pitch command
  der(theta_filt) = max(-theta_rate_limit, min(theta_rate_limit, (theta_cmd - theta_filt)));
  
  // Inner loop: Alpha command based on pitch error
  alpha_cmd = kp_alpha * (theta_filt - theta) + (theta_filt - gamma);
  
  // Velocity control
  thr_cmd = kv*(v_t - v); 

  // Actuator limits
  alpha = min(0.349, max(0, alpha_cmd));
  thr = min(1, max(0, thr_cmd));
  
  // Kinematic relationship
  theta = alpha + gamma;
  
  // Kinematics
  der(h) = if h > 0 or v * sin(gamma) > 0 then v * sin(gamma) else 0;
  der(x) = v * cos(gamma);
  x_t = 1000000;

  // Force balance
  Ft = T * cos(alpha) - D - W * sin(gamma) - B;
  Fl = L + T * sin(alpha) - W * cos(gamma) + N;
  
  if phase == 7 then
    der(gamma) = -gamma/10;
    der(v) = -v/100;
  else 
    Fl = m*v*der(gamma);
    Ft = m*der(v);
  end if;
  
  Fh = T * cos(alpha+gamma) - D * cos(gamma) - L * sin(gamma) - B;
  Fv = L * cos(gamma) + T * sin(alpha+gamma) - W - D * sin(gamma) + N;
  
  // Phase-based targets with altitude control
  if phase == 1 then // Takeoff roll
    v_t = 130;
    theta_t = 0.05;
    h_t = 0;
    altitude_hold = false;
    
  elseif phase == 2 then // Climb to 11000m
    v_t = 230; 
    theta_t = 0.314;       // Base pitch for climb (~18°)
    h_t = 11000;          // TARGET ALTITUDE
    altitude_hold = true; // ENABLE ALTITUDE HOLD
    
  elseif phase == 3 then // Cruise at 11000m
    v_t = 230;
    theta_t = 0.05;
    h_t = 11000;          
    altitude_hold = true; 
    
  elseif phase == 4 then // Descent
    v_t = 100;
    theta_t = -0.10;
    h_t = 100;           
    altitude_hold = true; 
    
  elseif phase == 5 then // Flare
    v_t = 0;
    theta_t = 0.01;
    h_t = 0;
    altitude_hold = false;
    
  else // Ground roll / stopped
    v_t = 0;
    theta_t = 0;
    h_t = 0;
    altitude_hold = false;
    
  end if;
  
annotation(
    experiment(StartTime = 0, StopTime = 8000, Tolerance = 1e-06, Interval = 0.2));
end PlaneDynamic;
