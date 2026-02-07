package PlaneDynamics2D
  package Interfaces
    package Force_Interfaces
      connector Force_Horizontal
        Real V;
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
    model Wings
      equation

      end Wings;

      model Mass
      equation

      end Mass;

      model Ground
      equation

      end Ground;

      model Kinematics
      equation

      end Kinematics;
    end AerodynamicSystem;

    
    model  PlaneBody
      Interfaces.Environment_Interfaces.General_Interfaces general_Interfaces annotation(
              Placement(transformation(origin = {-8, -6}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-8, -6}, extent = {{-10, -10}, {10, 10}})));
            // States
            Real thr(start = 0.0);
            Real alpha(start = 0.0);
            Real theta(start = 0.05);
            Real gamma(start = 0.0);
            Real v(start = 0.01);
            Real h(start = 0);
            Real x(start = 0);
            Real theta_filt(start = 0.0);
            //Real m_fuel(start=0);  // FIXED: Added start value
      // Phase tracker
            discrete Integer phase(start = 1);
            // Constants
            parameter Real T_gen = 150000;
            parameter Real g = 9.81;
            parameter Real s = 122.6;
            parameter Real Cl0 = 0.6;
            parameter Real Cl_alpha = 4.39;
            parameter Real Cd0 = 0.022;
            parameter Real k = 0.045;
            parameter Real kp_h = 0.00005;
            // Altitude loop gain (h → theta)
            parameter Real kp_theta = 2;
            // Pitch loop gain (theta → alpha)
            parameter Real kp_alpha = 5;
            // Alpha loop gain
            parameter Real kv = 0.5;
            parameter Real kb = 0.5;
            parameter Real theta_rate_limit = 0.05;
            //parameter Real m_tank = 20000;
      //parameter Real rate = 0.75;
      // Variables
            Real rho, Cl, Cd, m;
            Real L, D, T, W, B, N;
            Real Fl, Ft;
            Real Fv, Fh;
            Real h_t, theta_t, v_t, x_t;
            Real theta_base;
            // Base theta from altitude controller
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
            elsewhen (x_t - x) < 30000 and phase == 3 then
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
// der(m_fuel) = -rate*thr;
            general_Interfaces.throttle = thr;
            m = 50000 + max(0, 20000 - general_Interfaces.fuel_left);
// Aerodynamics
            rho = 1.225*exp(-h/8500);
            Cl = Cl0 + Cl_alpha*alpha;
            Cd = Cd0 + k*Cl^2;
            L = 0.5*rho*v^2*s*Cl;
            D = 0.5*rho*v^2*s*Cd;
            T = general_Interfaces.Thrust;
            W = m*g;
            B = if phase == 6 or phase == 7 then kb*(m*v^2)/2000 else 0;
            N = if phase == 1 or phase == 6 or phase == 7 then W*cos(gamma) - L + T*sin(alpha) else 0;
// Hierarchical control with altitude hold
            theta_base = if altitude_hold then max(-0.2, min(0.4, theta_t + kp_h*(h_t - h))) else theta_t;
// Middle loop: Pitch tracking
            theta_cmd = theta_base + kp_theta*(theta_base - theta);
// Rate limiting on pitch command
            der(theta_filt) = max(-theta_rate_limit, min(theta_rate_limit, (theta_cmd - theta_filt)));
// Inner loop: Alpha command based on pitch error
            alpha_cmd = kp_alpha*(theta_filt - theta) + (theta_filt - gamma);
// Velocity control
            thr_cmd = kv*(v_t - v);
// Actuator limits
            alpha = min(0.349, max(0, alpha_cmd));
            thr = min(1, max(0, thr_cmd));
// Kinematic relationship
            theta = alpha + gamma;
// Kinematics
            der(h) = if h > 0 or v*sin(gamma) > 0 then v*sin(gamma) else 0;
            der(x) = v*cos(gamma);
            x_t = 1000000;
// Force balance
            Ft = T*cos(alpha) - D - W*sin(gamma) - B;
            Fl = L + T*sin(alpha) - W*cos(gamma) + N;
            if phase == 7 then
              der(gamma) = -gamma/10;
              der(v) = -v/100;
            else
              Fl = m*v*der(gamma);
              Ft = m*der(v);
            end if;
            Fh = T*cos(alpha + gamma) - D*cos(gamma) - L*sin(gamma) - B;
            Fv = L*cos(gamma) + T*sin(alpha + gamma) - W - D*sin(gamma) + N;
    // Phase-based targets with altitude control
if phase == 1 then
// Takeoff roll
              v_t = 130;
              theta_t = 0.05;
              h_t = 0;
              altitude_hold = false;
            elseif phase == 2 then
// Climb to 11000m
              v_t = 230;
              theta_t = 0.314;
// Base pitch for climb (~18°)
              h_t = 11000;
// TARGET ALTITUDE
              altitude_hold = true;
    // ENABLE ALTITUDE HOLD
elseif phase == 3 then
// Cruise at 11000m
              v_t = 230;
              theta_t = 0.05;
              h_t = 11000;
              altitude_hold = true;
            elseif phase == 4 then
// Descent
              v_t = 100;
              theta_t = -0.10;
              h_t = 100;
              altitude_hold = true;
            elseif phase == 5 then
// Flare
              v_t = 0;
              theta_t = 0.01;
              h_t = 0;
              altitude_hold = false;
            else
// Ground roll / stopped
              v_t = 0;
              theta_t = 0;
              h_t = 0;
              altitude_hold = false;
            end if;
            annotation(
              experiment(StartTime = 0, StopTime = 8000, Tolerance = 1e-06, Interval = 0.2),
              Icon(graphics = {Ellipse(origin = {-2, -1}, extent = {{-12, 19}, {12, -19}})}));
    end PlaneBody;
  end Plane;

  package Phases
    model TakeOff
    
      Real v, h, v_t, h_t, theta, altitude_hold, B, N, x, x_t;
    
    algorithm
      when v > 80 and phase == 1 then
        phase := 2;
      elsewhen h > 11000 and phase == 2 then
        phase := 3;
      end when;
      
    equation
        
      if phase == 1 then
        v_t = 130;
        theta_t = 0.05;
        h_t = 0;
        altitude_hold = false;
        B = 0;
        N = W*cos(gamma) - L + T * sin(alpha);
      elseif phase == 2 then
        v_t = 230; 
        theta_t = 0.314;       
        h_t = 11000;          
        altitude_hold = true;
        B = 0;
        N = 0;
      else
        v_t = 230; 
        theta_t = 0.314;       
        h_t = 11000;          
        altitude_hold = true;
        B = 0;
        N = 0;
      end if;
    
    
    end TakeOff;

    model Cruise
    
      Real v, h, v_t, h_t, theta, altitude_hold, B, N, x, x_t;
    
    algorithm
      when (x_t - x) < 6000 and phase == 3 then
        phase := 4;
      end when;
      
    equation
        
      if phase == 3 then // Cruise at 11000m
        v_t = 230;
        theta_t = 0.05;
        h_t = 11000;          
        altitude_hold = true; 
        B = 0;
        N = 0;
      else
        v_t = 230; 
        theta_t = 0.05;       
        h_t = 11000;          
        altitude_hold = true;
        B = 0;
        N = 0;
      end if;

    end Cruise;

    model Landing
    
      Real v, h, v_t, h_t, theta, altitude_hold, B, N, x, x_t;
    
    algorithm
      when h <= 50 and h > 0 and phase == 4 then
        phase := 5;
      elsewhen h <= 0 and phase == 5 then
        phase := 6;
      elsewhen h <= 0 and v < 50 and phase == 6 then 
        phase := 7;
      end when;
      
    equation
  if phase == 4 then // Descent
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
        theta_t = 0.05;
        h_t = 0;
        altitude_hold = false;
        
      end if;
    end Landing;
  end Phases;

  model PlaneDynamic
  Plane.Engine engine1 annotation(
      Placement(transformation(origin = {4, -22}, extent = {{-10, -10}, {10, 10}})));
  Plane.PlaneBody planeBody annotation(
      Placement(transformation(origin = {-37, 29}, extent = {{-33, -33}, {33, 33}})));
  equation
  connect(planeBody.general_Interfaces, engine1.general_Interfaces) annotation(
      Line(points = {{-40, 27}, {2, 27}, {2, -22}}));
        annotation(
          experiment(StartTime = 0, StopTime = 8000, Tolerance = 1e-06, Interval = 0.2));
  end PlaneDynamic;
end PlaneDynamics2D;
