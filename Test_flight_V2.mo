model Test_flight
  // States
  Real thr(start=0.0);
  Real alpha(start=0.0);
  Real gamma(start=0.0);
  Real v(start=0);
  Real h(start=0);
  Real x(start=0);
  Real integral_v(start=0);
  
  // Constants
  parameter Real T_gen = 300000;
  parameter Real m = 60000;
  parameter Real g = 9.81;
  parameter Real s = 122.6;
  parameter Real Cl0 = 0.6;
  parameter Real Cl_alpha = 4.39;
  parameter Real Cd0 = 0.022;
  parameter Real k = 0.045;
  
  // Mission parameters
  parameter Real x_dest = 2000000;
  parameter Real h_t = 11000;
  parameter Real v_l = 80;
  parameter Real v_cruise = 250;
  parameter Real gamma_l = 0.314;
  parameter Real gamma_descent = -0.0524;
  parameter Real gamma_approach = -0.0524;
  
  // Ground operations
  parameter Real mu_brake = 0.4;
  
  // Control gains
  parameter Real Kp_h = 0.0005;
  parameter Real Kp_v = 0.1;
  parameter Real Ki_v = 0.0001;
  parameter Real Kp_gamma = 1.2;
  parameter Real tau_thr = 1;
  parameter Real tau_alpha = 0.5;
  
  // Variables
  Real rho, Cl, Cd, L, D, T, W;
  Real Fl, Ft, N;
  Boolean has_taken_off;
  Integer phase;
  
initial equation
  has_taken_off = false;
  
equation
  when h > 5.0 then
    has_taken_off = true;
  end when;
  
  // Aerodynamics
  rho = 1.225 * exp(-h / 8500);
  Cl = Cl0 + Cl_alpha * alpha;
  Cd = Cd0 + k * Cl^2;
  L = 0.5 * rho * v^2 * s * Cl;
  D = 0.5 * rho * v^2 * s * Cd;
  T = thr * T_gen;
  W = m * g;
  
  Ft = T * cos(alpha) - D - W * sin(gamma);
  Fl = L + T * sin(alpha) - W * cos(gamma);
  
  der(x) = v * cos(gamma);
  
  // ===== UNIFIED FLIGHT DYNAMICS + CONTROL (PHASES 1-9) =====
  
  if h <= 0 and not has_taken_off then
    // ===== PHASE 1: TAKEOFF GROUND ROLL =====
    phase = 1;
    N = 0;
    der(v) = Ft / m;
    der(gamma) = Fl / (m * v + 0.001);
    
    if Fl > W * cos(gamma) then
      der(h) = v * sin(gamma);
    else
      der(h) = 0;
    end if;
    
    der(alpha) = 0;
    der(thr) = (1.0 - thr) / tau_thr;
    der(integral_v) = v_l - v;
    
  elseif h < 0.95 * h_t and (x_dest - x) > (h_t / tan(abs(gamma_descent))) then
    // ===== PHASE 2: CLIMB =====
    phase = 2;
    N = 0;
    der(v) = Ft / m;
    der(gamma) = Fl / (m * v + 0.001);
    der(h) = v * sin(gamma);
    
    der(alpha) = (gamma_l + Kp_gamma * (gamma_l - gamma) - alpha) / tau_alpha;
    der(thr) = (min(1.0, max(0.0, 0.85 + Kp_v * (v_l + (v_cruise - v_l) * (h / h_t) - v) + Ki_v * integral_v)) - thr) / tau_thr;
    der(integral_v) = v_l + (v_cruise - v_l) * (h / h_t) - v;
    
  elseif (x_dest - x) > (h_t / tan(abs(gamma_descent))) then
    // ===== PHASE 3: CRUISE =====
    phase = 3;
    N = 0;
    der(v) = Ft / m;
    der(gamma) = Fl / (m * v + 0.001);
    der(h) = v * sin(gamma);
    
    der(alpha) = (Kp_h * (h_t - h) + Kp_gamma * (Kp_h * (h_t - h) - gamma) - alpha) / tau_alpha;
    der(thr) = (min(1.0, max(0.0, 0.7 + Kp_v * (v_cruise - v) + Ki_v * integral_v)) - thr) / tau_thr;
    der(integral_v) = v_cruise - v;
    
  elseif h > 1000 then
    // ===== PHASE 4: INITIAL DESCENT =====
    phase = 4;
    N = 0;
    der(v) = Ft / m;
    der(gamma) = Fl / (m * v + 0.001);
    der(h) = v * sin(gamma);
    
    der(alpha) = (gamma_descent + Kp_gamma * (gamma_descent - gamma) - alpha) / tau_alpha;
    der(thr) = (min(1.0, max(0.0, 0.35 + Kp_v * (150 + 100 * ((h - 1000) / 10000) - v) + Ki_v * integral_v)) - thr) / tau_thr;
    der(integral_v) = 150 + 100 * ((h - 1000) / 10000) - v;
    
  elseif h > 300 then
    // ===== PHASE 5: INTERMEDIATE APPROACH =====
    phase = 5;
    N = 0;
    der(v) = Ft / m;
    der(gamma) = Fl / (m * v + 0.001);
    der(h) = v * sin(gamma);
    
    der(alpha) = (gamma_approach + Kp_gamma * (gamma_approach - gamma) - alpha) / tau_alpha;
    der(thr) = (min(1.0, max(0.0, 0.30 + Kp_v * (100 + 50 * ((h - 300) / 700) - v) + Ki_v * integral_v)) - thr) / tau_thr;
    der(integral_v) = 100 + 50 * ((h - 300) / 700) - v;
    
  elseif h > 15 then
    // ===== PHASE 6: FINAL APPROACH =====
    phase = 6;
    N = 0;
    der(v) = Ft / m;
    der(gamma) = Fl / (m * v + 0.001);
    der(h) = v * sin(gamma);
    
    der(alpha) = (gamma_approach + Kp_gamma * (gamma_approach - gamma) - alpha) / tau_alpha;
    der(thr) = (min(1.0, max(0.0, 0.25 + Kp_v * (80 + 20 * ((h - 15) / 285) - v) + Ki_v * integral_v)) - thr) / tau_thr;
    der(integral_v) = 80 + 20 * ((h - 15) / 285) - v;
    
  elseif h > 0 or not has_taken_off then
    // ===== PHASE 7: FLARE =====
    phase = 7;
    N = 0;
    der(v) = Ft / m;
    der(gamma) = Fl / (m * v + 0.001);
    der(h) = v * sin(gamma);
    
    der(alpha) = (max(0.035, gamma_approach * (h / 15) + Kp_gamma * (gamma_approach * (h / 15) - gamma)) - alpha) / tau_alpha;
    der(thr) = (0.15 - thr) / tau_thr;
    der(integral_v) = v_l - v;
    
  elseif h <= 0 and has_taken_off and v > 0 then
    // ===== PHASE 8: LANDING ROLLOUT =====
    phase = 8;
    N = W * cos(gamma) - Fl;
    der(v) = (Ft - mu_brake * max(0, N)) / m;
    der(gamma) = -gamma / 100;
    der(h) = 0;
    der(alpha) = (0.035 - alpha) / tau_alpha;
    der(thr) = (0.0 - thr) / tau_thr;
    der(integral_v) = 0;
    
  else
    // ===== PHASE 9: STOPPED =====
    phase = 9;
    N = 0;
    der(v) = 0;
    der(gamma) = 0;
    der(h) = 0;
    der(alpha) = 0;
    der(thr) = 0;
    der(integral_v) = 0;
  end if;
  
  annotation(
    experiment(
      StartTime=0,
      StopTime=10000,
      Tolerance=1e-6,
      Interval=1
    )
  );

end Test_flight;
