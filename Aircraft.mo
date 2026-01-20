package Aircraft
  model aircraftEquation
  
    // Submodel instantiation
    aerodynamics aero;
    propulsion prop;
    massModel mass;
    
    // Parameters
    parameter Real Iy = 30000000;
    parameter Real controlGain = 5000000;
    
    //userinput
    //input Real delta_degree;//elevator deflection
    //input Real throttle;
    parameter Real delta_degree=10;
    parameter Real throttle=1;  
    //States
    Real v(start=70);
    Real gamma(start=0);//Flight path angle
    Real theta(start=0);//Pitch angle
    Real alpha;//Angle of attack
    Real delta;//elevator deflection
    Real q(start=0);
  
  equation
    aero.v = v;
    aero.alpha = alpha;
    prop.throttle = throttle;
    
    delta = delta_degree * Modelica.Constants.pi/180 * max(0, 1 - (alpha - 10*Modelica.Constants.pi/180) / (5*Modelica.Constants.pi/180));
    der(q) = (controlGain * delta) / Iy; // pitch acceleration
    der(theta) = q;                       // integrate pitch rate to get pitch angle
    alpha = theta - gamma;
  
  
    der(v)=(prop.Thrust-aero.Drag-mass.Weight*sin(gamma))/mass.m; //Acceleration
    der(gamma) = (aero.Lift - mass.Weight*cos(gamma)) / (mass.m*v);
  

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
    parameter Real rho = 1.225;// Air density [kg/m^3]
    parameter Real CL0 = 0.6;// Lift coefficient when alpha =0
    parameter Real CL_alpha = 4.39;// Lift coefficient for alpha per rad
    parameter Real CD= 0.04;// Drag coefficient
    input Real v;// Aircraft velocity [m/s]
    input Real alpha;//angle of attack, should not exceed 15 degrees
    output Real Lift;
    output Real Drag;
    Real CL;
  
  equation
    CL = CL0 + CL_alpha * alpha;
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

model status

  // Submodel instantiation
  aircraftEquation equa;

  Real height(start=0);
  Real range(start=0);
  Real gamma;
  Real v;
  
equation
  gamma = equa.gamma;
  v=equa.v;
  der(height) = v * sin(gamma);
  der(range) = v * cos(gamma);

end status;
end Aircraft;
