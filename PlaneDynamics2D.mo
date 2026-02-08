package PlaneDynamics2D
  package Interfaces
    package Force_Interfaces
      connector Force_Horizontal
        Real Gamma;
        //Plane's velocity
        flow Real F_h;
        //Plane's body_fixed horizontal Force
        annotation(
          Icon(graphics = {Ellipse(origin = {0, -2}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid, extent = {{-28, 28}, {28, -28}})}));
      
      end Force_Horizontal;

      connector Force_Vertical
        Real Gamma;
        //Plane's flight vector angle
        flow Real F_v;
        //Plane's body_fixed vertical Force
        annotation(
          Icon(graphics = {Ellipse(origin = {0, -2}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Solid, extent = {{-28, 28}, {28, -28}})}));
      
      end Force_Vertical;
    end Force_Interfaces;

    package Combustion_Interfaces
      connector Mass_Flow
        Real p;
        flow Real m_fuel_rate;
        annotation(
          Icon(graphics = {Ellipse(origin = {0, -2}, fillColor = {0, 170, 0}, fillPattern = FillPattern.Solid, extent = {{-28, 28}, {28, -28}})}));
      end Mass_Flow;

      connector Heat_Flow
        Real T;
        flow Real Q;
        annotation(
          Icon(graphics = {Ellipse(origin = {0, -2}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-28, 28}, {28, -28}})}));
      end Heat_Flow;
    end Combustion_Interfaces;

    package Environment_Interfaces
      connector General_Interfaces
        Real throttle;
        Real fuel_left;     
        Real Thrust;
        //Real density;
        //Real g;
      annotation(
              Icon(graphics = {Rectangle(fillPattern = FillPattern.Solid, extent = {{-26, 24}, {26, -24}})}));
      
      end General_Interfaces;
    end Environment_Interfaces;
  end Interfaces;

  package Environments
     
package EarthModel
      model Gravitation
        parameter Real g = 9.81;
      equation

      end Gravitation;

      model Wind
      equation

      end Wind;

      model Density
      equation

      end Density;
    end EarthModel;

    model Atmosphere
    equation

    end Atmosphere;
  end Environments;

  package Plane
    package EngineSystem
      model Fuel_Tank
        Interfaces.Combustion_Interfaces.Mass_Flow fuelOut annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}), iconTransformation(extent = {{-10, -10}, {10, 10}})));
        parameter Real fuel_rate = 1 "kg/s";
        parameter Real p0 = 101325 "Pa";
        Real thr, m_fuel, fuel_left;
      
      equation
        der(m_fuel) = thr*fuel_rate;
        fuel_left = m_fuel;
        fuelOut.m_fuel_rate = thr*fuel_rate;
        fuelOut.p = p0;
        annotation(
          Icon(graphics = {Rectangle(extent = {{-24, 32}, {24, -32}})}));
      end Fuel_Tank;

      model Combustion
        Interfaces.Combustion_Interfaces.Mass_Flow fuelIn annotation(
          Placement(transformation(origin = {-16, 30}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-32, 0}, extent = {{-10, -10}, {10, 10}})));
        Interfaces.Combustion_Interfaces.Heat_Flow heatOut annotation(
          Placement(transformation(origin = {12, -12}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {32, 0}, extent = {{-10, -10}, {10, 10}})));
        parameter Real LHV = 43e6 "J/kg";
      equation
        heatOut.Q = fuelIn.m_fuel_rate*LHV;
        annotation(
          Icon(graphics = {Ellipse(origin = {0, 2}, extent = {{-48, 32}, {48, -32}})}));
      end Combustion;

      model Nozzle
        Interfaces.Combustion_Interfaces.Heat_Flow heatIn annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}), iconTransformation(extent = {{-10, -10}, {10, 10}})));
        parameter Real T_amb = 500;
        parameter Real efficiency = 0.8 "0..1";
        parameter Real v_e = 300 "m/s effective
                                     exhaust velocity";
        output Real Thrust;
        Real Q_waste;
      equation
// Waste shares the same temperature level
        heatIn.T = T_amb;
        Q_waste = (1 - efficiency)*heatIn.Q;
// Convert useful thermal power to thrust
        Thrust = 2*(efficiency*heatIn.Q)/v_e;
        annotation(
          Icon(graphics = {Rectangle(fillColor = {170, 170, 0}, extent = {{-62, 18}, {62, -18}})}));
      end Nozzle;
    end EngineSystem;

model Engine
Interfaces.Environment_Interfaces.General_Interfaces general_Interfaces annotation(
    Placement(transformation(origin = {-38, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-28, 0}, extent = {{-44, -44}, {44, 44}})));
  EngineSystem.Fuel_Tank fuel_tank annotation(
    Placement(transformation(origin = {-53, 45}, extent = {{-23, -23}, {23, 23}})));
  EngineSystem.Combustion combustion annotation(
    Placement(transformation(origin = {-2, 12}, extent = {{-22, -22}, {22, 22}})));
  EngineSystem.Nozzle nozzle annotation(
    Placement(transformation(origin = {49, -35}, extent = {{-29, -29}, {29, 29}})));


equation
  connect(fuel_tank.fuelOut, combustion.fuelIn) annotation(
    Line(points = {{-52, 46}, {-10, 46}, {-10, 12}}));
  connect(combustion.heatOut, nozzle.heatIn) annotation(
    Line(points = {{6, 12}, {50, 12}, {50, -34}}));
  fuel_tank.thr = general_Interfaces.throttle;
  general_Interfaces.Thrust = nozzle.Thrust;
  general_Interfaces.fuel_left = fuel_tank.fuel_left;
  annotation(
    Icon(graphics = {Polygon(origin = {10, 0}, fillColor = {138, 138, 138}, fillPattern = FillPattern.Vertical, points = {{-50, 20}, {-50, -20}, {50, -40}, {50, 40}, {-50, 20}, {-50, 20}})}));
end Engine;

    package AerodynamicSystem
      import PlaneDynamics2D.Interfaces.Force_Interfaces.*;
      
      model Wings
        // Inputs (signals)
        input Real alpha;
        input Real gamma;
        input Real v;
        input Real h;
      
        parameter Real s = 122.6;
        parameter Real Cl0 = 0.6;
        parameter Real Cl_alpha = 4.39;
        parameter Real Cd0 = 0.022;
        parameter Real k = 0.045;
      
        // Outputs only for debugging
        output Real L;
        output Real D;
      
        Real rho, Cl, Cd;
      
        Force_Horizontal force_horizontal;
        Force_Vertical force_vertical;
      
      equation
        rho = 1.225 * exp(-h / 8500);
        Cl  = Cl0 + Cl_alpha * alpha;
        Cd  = Cd0 + k * Cl^2;
      
        L = 0.5 * rho * v^2 * s * Cl;
        D = 0.5 * rho * v^2 * s * Cd;
      
        // Contribute aerodynamic forces to the sum
        // Sign convention:
        //   +F_hh = forward, drag is backwards
        //   +F_vv = upward, lift is upward
        force_horizontal.F_h = -D;
        force_vertical.F_v = +L;
      end Wings;
    
      model Mass
        input Real thr;
      
        parameter Real g = 9.81;
        parameter Real m_empty = 50000;
        parameter Real m_tank  = 20000;
        parameter Real rate    = 0.75;
      
        Real m_fuel(start=0.1);
        output Real m;
        output Real W;
      
        Force_Vertical force_vertical; // only vertical force contribution
        Force_Horizontal force_horizontal;
      
      equation
        der(m_fuel) = -rate * thr;
        m = m_empty + smooth(0, max(0, m_tank + m_fuel));
        W = m * g;
      
        // Weight acts downward => negative vertical contribution
      
        force_vertical.F_v = -W*cos(force_vertical.Gamma);
        force_horizontal.F_h = -W*sin(force_horizontal.Gamma);
      end Mass;
    
      model Ground
        input Real m;
        input Real v;
        input Real gamma;
        input Real alpha;
        input Real T;
        input Real L;
        input Boolean onGround;
      
        parameter Real kb = 0.5;
      
        Real B;
        Real N;
        Real W;
      
        Force_Horizontal force_horizontal;
        Force_Vertical force_vertical;
      
      equation
        W = m * 9.81;
      
        // Simple brake model
        B = if onGround then kb * (m * v^2) / 2000 else 0;
      
        // Simple normal force estimate (only meaningful on ground)
        N = if onGround then (W*cos(gamma) - L + T*sin(alpha)) else 0;
      
        // Braking opposes forward direction => negative horizontal force
        force_horizontal.F_h = -B;
      
        // Normal is upward => positive vertical force
        force_vertical.F_v = +N;
      end Ground;
    
      model Kinematics
        // States
        Real thr(start=0.0);
        Real alpha(start=0.0);
        Real theta(start=0.05);
        Real gamma(start=0.05);  // Fix gamma at initialization
        Real v(start=0.01);     // Fix v at initialization
        Real h(start=0);
        Real x(start=0);
        Real theta_filt(start=0.01);
        parameter Real v_eps = 0.5;
        parameter Real T_alpha = 0.1 "Actuator time constant";
      
        // Phase tracker
        discrete Integer phase(start=1);
      
        // Parameters
        parameter Real T_gen = 240000;
      
        // Control parameters
        parameter Real kp_h = 0.00005;
        parameter Real kp_theta = 2;
        parameter Real kp_alpha = 5;
        parameter Real kv = 0.5;
        parameter Real theta_rate_limit = 0.05;
      
        // Targets
        Real h_t, theta_t, v_t, x_t;
        Boolean altitude_hold;
      
        // Commands
        Real theta_base;
        Real theta_cmd;
        Real thr_cmd;
        Real alpha_cmd;
      
        // Inputs from submodels (signals)
        input Real m;
      
        // Inputs for ground logic
        output Boolean onGround;
      
        // Outputs to other submodels (signals)
        output Real T;
      
        // Force ports (these will SUM contributions from all connected components)
        Force_Horizontal force_horizontal;
        Force_Vertical force_vertical;
      
        // Internal force totals (after summation)
        Real Fh_tot;
        Real Fv_tot;
      
      algorithm
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
        // Decide "on ground" phases (you can refine)
        onGround = (phase == 1) or (phase == 6) or (phase == 7);
      
        // Engine thrust from throttle (simple)
        T = thr * T_gen;
      
        // Guidance targets
        if phase == 1 then
          v_t = 150; theta_t = 0.05; h_t = 0; altitude_hold = false;
        elseif phase == 2 then
          v_t = 230; theta_t = 0.314; h_t = 11000; altitude_hold = true;
        elseif phase == 3 then
          v_t = 230; theta_t = 0.05;  h_t = 11000; altitude_hold = true;
        elseif phase == 4 then
          v_t = 100; theta_t = -0.10; h_t = 100; altitude_hold = true;
        elseif phase == 5 then
          v_t = 0; theta_t = 0.01; h_t = 0; altitude_hold = false;
        else
          v_t = 0; theta_t = 0.05; h_t = 0; altitude_hold = false;
        end if;
      
        // Control
        theta_base = if altitude_hold then max(-0.2, min(0.4, theta_t + kp_h*(h_t - h))) else theta_t;
        theta_cmd  = theta_base + kp_theta*(theta_base - theta);
        der(theta_filt) = max(-theta_rate_limit, min(theta_rate_limit, (theta_cmd - theta_filt)));
        alpha_cmd = kp_alpha * (theta_filt - theta) + (theta_filt - gamma);
        T_alpha * der(alpha) = min(0.349, max(0, alpha_cmd)) - alpha;
      
        thr_cmd = kv*(v_t - v);
        thr = min(1, max(0, thr_cmd));
      
        // Kinematic relationship
        theta = alpha + gamma;
      
        // Position kinematics
        der(h) = if h > 0 or v*sin(gamma) > 0 then v*sin(gamma) else 0;
        der(x) = v*cos(gamma);
        x_t = 1000000;
      
        // Summed forces from ports
        // NOTE: with flow variables, the "sum to zero" rule means
        // the net force acting on this model is -Force_Horizontal.F_hh and -Force_Vertical.F_vv.
        force_vertical.Gamma = gamma;
        force_horizontal.Gamma = gamma;
        Fh_tot = -force_horizontal.F_h + T*cos(alpha);
        Fv_tot = -force_vertical.F_v + T*sin(alpha); 
      
        // Dynamics
        if phase == 7 then
          der(gamma) = -gamma/10;
          der(v) = -v/100;
        else
          // Horizontal acceleration
          m*der(v) = Fh_tot;
      
          // Flight-path angle dynamics (small-angle-ish force balance)
          
          m*v*der(gamma) = Fv_tot;
        end if;
      
        annotation(
          experiment(StartTime = 0, StopTime = 8000, Tolerance = 1e-06, Interval = 0.2));
      end Kinematics;
    
      model Modelli
      
        Ground ground;
        Mass mass;
        Wings wings;
        Kinematics kinematics;
      
      equation
        // Signal wiring
        wings.alpha = kinematics.alpha;
        wings.gamma = kinematics.gamma;
        wings.v     = kinematics.v;
        wings.h     = kinematics.h;
      
        mass.thr = kinematics.thr;
      
        ground.m = mass.m;
        ground.v = kinematics.v;
        ground.gamma = kinematics.gamma;
        ground.alpha = kinematics.alpha;
        ground.T = kinematics.T;
        ground.L = wings.L;
        ground.onGround = kinematics.onGround;
      
        // Feed mass into kinematics
        kinematics.m = mass.m;
      
        // Force wiring (flows sum)
        connect(wings.force_horizontal, kinematics.force_horizontal);
        connect(wings.force_vertical, kinematics.force_vertical);
      
        connect(mass.force_horizontal, kinematics.force_horizontal);
        connect(mass.force_vertical, kinematics.force_vertical);
      
      
        connect(ground.force_horizontal, kinematics.force_horizontal);
        connect(ground.force_vertical, kinematics.force_vertical);
        annotation(
          experiment(StartTime = 0, StopTime = 8000, Tolerance = 1e-06, Interval = 0.2));
      
      end Modelli;
    end AerodynamicSystem;
  end Plane;

  package Phases
    model TakeOff
    
      discrete Integer phase = 1;
    equation
    
    
    end TakeOff;

    model Cruise
    
    equation

    end Cruise;

    model Landing
    
    equation
    
    end Landing;
  end Phases;
end PlaneDynamics2D;
