package Aircraft
  model aircraftEquation
  
    // Submodel instantiation
    aerodynamics aero;
    propulsion prop;
    massModel mass;
    
    // Parameters
    parameter Real Iy = 500;
    parameter Real controlGain = 500;
    
    //userinput
    input Real delta;
    input Real throttle;
    
    //States
    Real v(start=30);
    Real gamma(start=0);
    Real theta(start=0);
    Real alpha;
    
    //Intermediate variables
    Real q(start=0);
  
  equation
    aero.v = v;
    aero.alpha = alpha;
    prop.throttle = throttle;
    
    der(q) = (controlGain * delta) / Iy; // pitch acceleration
    der(theta) = q;                       // integrate pitch rate to get pitch angle
    alpha = theta - gamma;
  
    der(v)=(prop.Thrust-aero.Drag-mass.Weight*sin(gamma))/mass.m; //Acceleration
    der(gamma) = (aero.Lift - mass.Weight*cos(gamma)) / (mass.m*v);
  

  end aircraftEquation;
  
  model propulsion
    parameter Real Tmax = 5000; // Maximum thrust [N]
    input Real throttle;// 0..1
    output Real Thrust;
  
  equation
    Thrust = throttle * Tmax;
  
  end propulsion;
  
  model aerodynamics
    parameter Real S = 10;// Wing area [m^2]
    parameter Real rho = 1.225;// Air density [kg/m^3]
    parameter Real CL0 = 0.2;// Lift coefficient when alpha =0
    parameter Real CL_alpha = 5;// Lift coefficient for alpha per rad
    parameter Real CD= 0.03;// Drag coefficient
    input Real v;// Aircraft velocity [m/s]
    input Real alpha;//angle of attack
    output Real Lift;
    output Real Drag;
    Real CL;
  
  equation
    CL = CL0 + CL_alpha * alpha;
    Lift=0.5*rho*v^2*S*CL;// Lift formula
    Drag=0.5*rho*v^2*S*CD;// Drag formula
  
  end aerodynamics;
  
  model massModel
    parameter Real m = 2000;
    constant Real g = 9.81;
    output Real Weight;
  
  equation
    Weight=m*g;
  
  end massModel;
end Aircraft;
