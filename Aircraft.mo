package Aircraft
  model aircraftEquation
  
   // Submodel instantiation
    aerodynamics aero;
    propulsion prop;
    massModel mass;
  
  equation
    throttle = 0.8;
    aero.v = v;
    prop.throttle = throttle;
    
     //Acceleration

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
    parameter Real CL = 0.5;// Lift coefficient
    parameter Real CD= 0.03;// Drag coefficient
    input Real v;// Aircraft velocity [m/s]
    output Real Lift;
    output Real Drag;
  
  equation
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
