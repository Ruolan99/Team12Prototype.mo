model idk_anymore

    // States
    Real thr(start=0.0);
    Real alpha(start=0.0);
    Real gamma(start=0.0);
    Real v(start=0.01);
    Real h(start=0);
    Real x(start=0);
    
    // Phase tracker (discrete to prevent chattering)
    discrete Integer phase(start=1);
    
    // Constants
    parameter Real T_gen = 150000;
    parameter Real m = 70000;
    parameter Real g = 9.81;
    parameter Real s = 122.6;
    parameter Real Cl0 = 0.6;
    parameter Real Cl_alpha = 4.39;
    parameter Real Cd0 = 0.022;
    parameter Real k = 0.045;
    parameter Real kp = 1;
    parameter Real kv = 0.5;
    parameter Real kb = 5; 
    
    // Variables
    Real rho, Cl, Cd; 
    Real L, D, T, W, B, N; //Forces
    Real Fl, Ft; //Balance Force Body Fixed
    Real Fv, Fh; //Balance Force Frame Fixed
    Real gamma_t, v_t, x_t;
    Real alpha_cmd;
    Real thr_cmd;

algorithm
  // Phase transitions (only move forward, never backward)
  when v > 80 and phase == 1 then
    phase := 2;
  elsewhen h > 11000 and phase == 2 then
    phase := 3;
  elsewhen (x_t - x) < 6000 and phase == 3 then
    phase := 4;
  elsewhen h <= 1000 and h > 0 and phase == 4 then
    phase := 5;
  elsewhen h <= 0 and phase == 5 then
    phase := 6;
  elsewhen h <= 0 and v < 20 and phase == 6 then 
    phase := 7;
  end when;

equation
  // Aerodynamics
  rho = 1.225 * exp(-h / 8500);
  Cl = Cl0 + Cl_alpha * alpha;
  Cd = Cd0 + k * Cl^2;
  L = 0.5 * rho * v^2 * s * Cl;
  D = 0.5 * rho * v^2 * s * Cd;
  T = thr * T_gen;
  W = m * g;
  B = if phase == 6 or phase == 7 then kb*(m*v^2)/2000 else 0;
  N = if phase == 1 or phase == 6 or phase == 7  then W*cos(gamma) - L + T * sin(alpha) else 0;

  alpha_cmd = kp*(gamma_t - gamma);
  thr_cmd = kv*(v_t - v); 

  alpha = min(0.349, max(0, alpha_cmd));
  thr = min(1, max(0, thr_cmd));
  
  // Kinematics with ground constraint
  der(h) = if h > 0 or v * sin(gamma) > 0 then v * sin(gamma) else 0;
  der(x) = v * cos(gamma);
  x_t = 1000000;

  // Force balanced
  Ft = T * cos(alpha) - D - W * sin(gamma) - B;
  Fl = L + T * sin(alpha) - W * cos(gamma) + N;
  
  //Stop
  if phase == 7 then
    der(gamma) = -gamma/10; //Fl = 0
    der(v) = -v/100;        //Ft = 0
  else 
    Fl = m*v*der(gamma);
    Ft = m*der(v);
  end if;
  
  Fh = T * cos(alpha+gamma) - D * cos(gamma) - L * sin(gamma) - B;
  Fv = L * cos(gamma) + T * sin(alpha+gamma) - W - D * sin(gamma) + N;
  
  // Phase-based targets
  if phase == 1 then //Takeoff -> Climb
    v_t = 130;
    gamma_t = 0;
    
  elseif phase == 2 then //Climb -> Cruise
    v_t = 230; 
    gamma_t = 0.314;
    
  elseif phase == 3 then //
    v_t = 230;
    gamma_t = 0;
    
  elseif phase == 4 then
    v_t = 100;
    gamma_t = -0.18;
    
  elseif phase == 5 then
    v_t = 0;
    gamma_t = 0;
    
  else
    v_t = 0;
    gamma_t = 0;
    
  end if;
  
annotation(
    experiment(StartTime = 0, StopTime = 8000, Tolerance = 1e-06, Interval = 0.2));
end idk_anymore;
