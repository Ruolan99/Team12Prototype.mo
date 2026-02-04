package Aircraft
  model aircraftEquation
  
    // Submodel instantiation
    aerodynamics aero;
    propulsion prop;
    massModel mass;
    
    // Parameters
  
    parameter Real controlGain = 5000000;
    parameter Real K_alpha = 1;
    parameter Real tau_alpha = 10;
    parameter Real tau_gamma = 10;
    
    //references
    Real alpha_ref;
    Real gamma_ref;
    
    parameter Real h_switch = 10000;
    parameter Real alpha_climb  = 7 * Modelica.Constants.pi/180;
    parameter Real alpha_cruise = 3 * Modelica.Constants.pi/180;
    parameter Real gamma_climb  = 5 * Modelica.Constants.pi/180;
    parameter Real gamma_cruise = 0 * Modelica.Constants.pi/180;
  
    parameter Real delta_degree=10;
    parameter Real throttle=1;
    
    
    //States
    Real v(start=70);
    Real gamma(start=0);//Flight path angle
    Real theta(start=0);//Pitch angle
    Real alpha;//Angle of attack
    Real delta;//elevator deflection
    Real height;
    Real range;
  
  equation
    aero.v = v;
    aero.height = height;
    aero.alpha = alpha;
    aero.gamma = gamma;
    aero.Weight = mass.Weight;
    prop.throttle = throttle;
    
    der(height) = v * sin(gamma);
    der(range) = v * cos(gamma);
    
    alpha_ref = if height < h_switch
              then alpha_climb
              else alpha_cruise;
    der(alpha) = (alpha_ref - alpha) / tau_alpha;
    
    gamma_ref = if height < h_switch
              then gamma_climb
              else gamma_cruise;
    der(gamma) = (gamma_ref - gamma) / tau_gamma;
    
    delta = K_alpha * (alpha_ref - alpha);
    theta = alpha + gamma;
  
  

  end aircraftEquation;
  
  model propulsion
    parameter Real Tmax = 1000000; // Maximum thrust [N]
    input Real throttle;// 0..1
    output Real Thrust;
  
  equation
    Thrust = throttle * Tmax;
  
  end propulsion;
  
  model aerodynamics
    parameter Real S = 400;// Wing area [m^2]
    Real rho;// Air density [kg/m^3]
    parameter Real CL0 = 0.6;// Lift coefficient when alpha =0
    parameter Real CL_alpha = 4.39;// Lift coefficient for alpha per rad
    parameter Real CD= 0.04;// Drag coefficient
    Real CL;
    
    input Real gamma;
    input Real alpha; 
    input Real Weight;
    input Real height;
    output Real Lift;
    output Real Drag;
    output Real v;
  
  
  equation 
    CL = CL0 + CL_alpha * alpha; 
    v = sqrt(2 * Weight * cos(gamma) / (rho * S * CL));
    rho = 1.225 * exp(-height/8500);
    Lift=0.5*rho*v^2*S*CL;// Lift formula
    Drag=0.5*rho*v^2*S*CD;// Drag formula
  
  end aerodynamics;
  
  model massModel
    parameter Real m = 300000;
    constant Real g = 9.81;
    output Real Weight;
  
  equation
    Weight=m*g;
  
  end massModel;
end Aircraft;
