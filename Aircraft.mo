package Aircraft

  package Connectors 
    connector FlightState
      Real v;
      Real gamma;
      Real alpha;
      Real theta;
      Real h;
      Real range;
      Real weight;
      Real Drag;
      Real thrust_total;
    end FlightState;

    connector ForceBus
      Real dummy;        // potential（占位）
      flow Real Fx;
      flow Real Fz;
    end ForceBus;
  end Connectors;

  package Components
    model Aerodynamics
      parameter Real S = 400;
      parameter Real CL0 = 0.6;
      parameter Real CL_alpha = 4.39;
      parameter Real CD = 0.04;
    
      input Aircraft.Connectors.FlightState state;
      output Aircraft.Connectors.ForceBus forces;
      output Real v;
      output Real Drag;
    
    equation
      v = sqrt(2*state.weight*cos(state.gamma)/(1.225*exp(-state.h/8500)*S*(CL0 + CL_alpha*state.alpha)));
    
      Drag = 0.5*1.225*exp(-state.h/8500)*v^2*S*CD;
      
      forces.Fx = Drag;
      forces.Fz = 0.5*1.225*exp(-state.h/8500)*v^2*S*(CL0 + CL_alpha*state.alpha) - state.weight*cos(state.gamma); 

    end Aerodynamics;

    model MassModel
      parameter Real m = 300000;
      constant Real g = 9.81;
    
      output Aircraft.Connectors.FlightState state;
    
    equation
      state.weight = m * g;
    end MassModel;

    model Propulsion
      input Aircraft.Connectors.FlightState state;
      output Real throttle;
    
      Real Thrust_req;
    equation
      Thrust_req =
        max(state.weight / 9.81 * der(state.v) + state.Drag, 0);
    
      throttle = min(Thrust_req / state.thrust_total, 1);
    end Propulsion;

    model Engine
      parameter Real Tmax = 1e6;
    
      input Real throttle;   // 0..1
      output Aircraft.Connectors.ForceBus forces;
    
    equation
      forces.Fx = throttle * Tmax;
      forces.Fz = 0;
    end Engine;
  end Components;

  model AircraftSystem
    // -------------------------
    // Connectors
    Aircraft.Connectors.FlightState flightState;
    Aircraft.Connectors.ForceBus totalForces;
  
    // -------------------------
    // Components
    Components.MassModel mass;
    Components.Aerodynamics aero;
    Components.Propulsion prop;
    Components.Engine eng; 
  
    // -------------------------
    // States
    Real v;
  
    // -------------------------
    // Control parameters
    parameter Real tau_alpha = 10;
    parameter Real tau_gamma = 10;
    parameter Real h_switch  = 10000;
  
    parameter Real alpha_climb  = 7*Modelica.Constants.pi/180;
    parameter Real alpha_cruise = 3*Modelica.Constants.pi/180;
    parameter Real gamma_climb  = 5*Modelica.Constants.pi/180;
    parameter Real gamma_cruise = 0;
  
    Real alpha_ref;
    Real gamma_ref;
  
  equation
    // -------------------------
    // Mass
    connect(mass.state, flightState);
  
    // -------------------------
    // Aerodynamics → Forces
    connect(flightState, aero.state);
    connect(aero.forces, totalForces);
  
    // -------------------------
    // Propulsion
    connect(flightState, prop.state);
  
    // -------------------------
    // Engine
    eng.throttle = prop.throttle;
  
    // -------------------------
    // Thrust capability
    flightState.thrust_total = eng.Tmax;
  
    // -------------------------
    // Longitudinal dynamics
    der(v) = totalForces.Fx / mass.m;
    flightState.v = v;
  
    der(flightState.h)     = v * sin(flightState.gamma);
    der(flightState.range) = v * cos(flightState.gamma);
  
    // -------------------------
    // Simple flight control
    alpha_ref =
      if flightState.h < h_switch then alpha_climb else alpha_cruise;
    der(flightState.alpha) =
      (alpha_ref - flightState.alpha)/tau_alpha;
  
    gamma_ref =
      if flightState.h < h_switch then gamma_climb else gamma_cruise;
    der(flightState.gamma) =
      (gamma_ref - flightState.gamma)/tau_gamma;
  
    flightState.theta =
      flightState.alpha + flightState.gamma;
  
  end AircraftSystem;
end Aircraft;
