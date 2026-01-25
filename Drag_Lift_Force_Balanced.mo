model Drag_Lift_Force_Balanced
  // universal
  parameter Real g = 9.80665;
  Real rho;

  // Lift
  parameter Real S = 122.6;
  parameter Real CL0 = 0.6;
  parameter Real CL_alpha = 4.39;
  parameter Real CL_max = 1.5;
  parameter Real alpha_max = 0.26;
  parameter Real gamma_max = 0.43;

  Real CL, L;
  Real alpha(start = 0);        // AoA (dynamic)
  Real theta;                   // Pitch
  Real gamma(start = 0);        // Flight path angle
  Real h(start = 0);

  // Drag
  parameter Real CD0 = 0.022;
  parameter Real k = 0.045;
  Real CD, D;

  // Thrust
  parameter Real T_gen = 300000;
  Real T, s(start = 0);

  // throttle (dynamic)
  Real Thr(start = 1);

  // Speed
  Real V(start = 0.01);

  // Weight
  parameter Real M = 69000;
  Real W;

  // Check
  Real Ft, Fl;

  // === Commands + actuator dynamics ===
  parameter Real tauThr   = 2.0;   // throttle lag [s]
  parameter Real tauAlpha = 0.5;   // AoA response lag [s]

  Real Thr_cmd;
  Real alpha_cmd;

  // Targets
  parameter Real V_ref = 260;      // speed target [m/s]
  parameter Real h_ref = 11000;    // altitude target [m]

  // Simple gains (tune)
  parameter Real Kt = 0.003;       // throttle gain for speed error
  parameter Real Ka = 0.00002;     // alpha gain for altitude error

  // === Soft limit parameters ===
  parameter Real V_max = 260;      // speed cap [m/s]
  parameter Real dV    = 2.0;      // smoothing width [m/s] around V_max
  parameter Real dG    = 0.02;     // smoothing width [rad] around gamma_max

  // Smooth “gates” (0..1) that turn dynamics off beyond limits
  Real gateV;      // ~1 below V_max, ~0 above V_max
  Real gateG;      // ~1 below gamma_max, ~0 above gamma_max

  // Helper: smooth step using tanh (no events)
  function smoothGate
    input Real x;
    input Real xMax;
    input Real dx;
    output Real y;
  algorithm
    // y ~ 1 when x << xMax, y ~ 0 when x >> xMax
    y := 0.5 * (1 - tanh((x - xMax) / dx));
  end smoothGate;

  // Helper: clamp (hard clamp is OK here because it’s algebraic, not an if-event on states)
  function clamp
    input Real u;
    input Real uMin;
    input Real uMax;
    output Real y;
  algorithm
    y := min(uMax, max(uMin, u));
  end clamp;

equation
  // Pitch kinematics
  theta = gamma + alpha;

  // Density
  rho = 1.225 * exp(-h / 8500);

  // Altitude
  der(h) = V * sin(gamma);
  
  // Distance
  der(s) = V * cos(gamma);

  // Weight
  W = M * g;

  // === Command laws (simple placeholders) ===
  // Speed via throttle (incremental)
  Thr_cmd = clamp(Thr + Kt*(V_ref - V), 0.0, 1.0);

  // Altitude via AoA (very simplified)
  alpha_cmd = clamp(Ka*(h_ref - h), -alpha_max, alpha_max);

  // === Actuator dynamics (Option 2) ===
  der(Thr)   = (Thr_cmd   - Thr)   / tauThr;
  der(alpha) = (alpha_cmd - alpha) / tauAlpha;

  // Thrust
  T = Thr * T_gen;

  // Aero coefficients
  CL = min(CL_max, CL0 + CL_alpha*alpha);
  CD = CD0 + k*CL^2;

  // Forces
  L = 0.5 * rho * V^2 * S * CL;
  D = 0.5 * rho * V^2 * S * CD;

  // Force balances
  Ft = T*cos(alpha) - D - W*sin(gamma);
  Fl = L + T*sin(alpha) - W*cos(gamma);

  // === Soft gates for BOTH limits ===
  gateV = smoothGate(V, V_max, dV);
  gateG = smoothGate(gamma, gamma_max, dG);

  // === Apply gates smoothly (no if-switching) ===
  M*der(V)        = gateV * Ft;
  M*V*der(gamma)  = gateG * Fl;

  annotation(
    experiment(
      StartTime=0,
      StopTime=1000,
      Tolerance=1e-6,
      Interval=1
    )
  );
end Drag_Lift_Force_Balanced;
