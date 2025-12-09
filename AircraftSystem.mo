package AircraftSystem
  package Energy
    connector mainForce
      flow Real Ft, Fl, Fd, Fw;
    end mainForce;

    connector energyConversion
      flow Real KE, PE;
    end energyConversion;
  end Energy;

  package Plane
    model Mass_model
      //A320neo
      parameter Real m = 2000;
      parameter Real g = 9.81;
      Real Weight;
    equation
      Weight = m*g;
    end Mass_model;

    model Aerodynamics
      parameter Real S = 122.4;
      // Wing area [m^2]
      parameter Real Wingspan = 35.80;
      parameter Real rho = 1.225;
      // Air density [kg/m^3]
      parameter Real CL = 0.5;
      // Lift coefficient
      parameter Real CD = 0.03;
      // Drag coefficient
      Real v;
      // Aircraft velocity [m/s]
      Real Lift, Drag;
    equation
      Lift = 0.5*rho*v^2*S*CL;
// Lift formula
      Drag = 0.5*rho*v^2*S*CD;
// Drag formula
    end Aerodynamics;

    model Propulsion
      parameter Real Tmax = 5000;
      // Maximum thrust [N]
      Real Throttle;
      // 0..1
      Real Thrust;
    equation
      Thrust = throttle*Tmax;
    end Propulsion;

    model Fuel_tank
      Environment.environmentInterfaces environmentInterfaces1 annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}), iconTransformation(extent = {{-10, -10}, {10, 10}})));
      parameter Real Max_fuel_capacity = 29.6;
      //[m^3]
      parameter Real Max_fuel_mass = 237000;
      //[kg]
      parameter Real Fuel_density = 800;
      //[kg/m^3]
      Real Fuel_mass;
      parameter Real takeoff = 0.35;
      //[kg/s] one engine
      //parameter Real climb = 0.28;
      parameter Real cruising = 0.25;
      parameter Real landing = 0.08;
      Real fuel_flow;
    equation
      der(Fuel_mass) = fuel_flow;
      if environmentInterfaces1.phase == 1 then
// Takeoff
        fuel_flow = takeoff;
      elseif environmentInterfaces1.phase == 2 then
// Cruising
        fuel_flow = cruising;
      elseif environmentInterfaces1.phase == 3 then
// Landing
        fuel_flow = landing;
      else
        fuel_flow = 0;
// default/fail-safe
      end if;
      annotation(
        Icon(graphics = {Ellipse(origin = {0, -1}, extent = {{-58, 43}, {58, -43}})}),
        Diagram);
    end Fuel_tank;

    model Engine
    equation

    end Engine;

    model Capacity
      parameter Real passenger_cap = 195;
      //[People]
      parameter Real passenger_avg_weight = 65;
      //[kg]
      parameter Real storage_cap = 0;
      //[kg]
    equation

    end Capacity;

    connector planeInterfaces
      annotation(
        Icon);
    end planeInterfaces;

    model Airplane_model
      // Submodel instantiation
      Aerodynamics aero;
      Propulsion prop;
      Mass_model mass;
    equation
      throttle = 0.8;
      aero.v = v;
      prop.throttle = throttle;
    end Airplane_model;
  end Plane;

  package Environment
    model Atmosphere
      Real rho;
    equation

    end Atmosphere;

    model Wind
    equation

    end Wind;

    model Passenger
    equation

    end Passenger;

    connector environmentInterfaces
      Integer phase;
      //1=Takeoff, 2=Cruising, 3=Landing"
      annotation(
        Icon(graphics = {Rectangle(origin = {12, 1}, extent = {{-32, 25}, {32, -25}})}));
    end environmentInterfaces;

    model Enviroment_model
    equation

    end Enviroment_model;

    package FlightPhases
      model Takeoff
        environmentInterfaces environmentInterfaces1 annotation(
          Placement(transformation(origin = {2, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {2, 0}, extent = {{-10, -10}, {10, 10}})));
      equation
        environmentInterfaces1.phase = 1;
      annotation(
          Icon(graphics = {Rectangle(origin = {7, -4}, extent = {{-51, 14}, {51, -14}})}));
end Takeoff;

      model Cruising
        AircraftSystem.Environment.environmentInterfaces env;
      equation
        env.phase = 2;
      end Cruising;

      model Landing
        AircraftSystem.Environment.environmentInterfaces env;
      equation
        env.phase = 3;
      end Landing;
    end FlightPhases;
  end Environment;

  package Simulation
    model Ex
      Plane.Fuel_tank fuel_tank annotation(
        Placement(transformation(origin = {11, 25}, extent = {{-43, -43}, {43, 43}})));
      Environment.FlightPhases.Takeoff takeoff annotation(
        Placement(transformation(origin = {14, -64}, extent = {{-40, -40}, {40, 40}})));
    equation
  connect(takeoff.environmentInterfaces1, fuel_tank.environmentInterfaces1) annotation(
        Line(points = {{14, -64}, {12, -64}, {12, 26}}));
    end Ex;
  end Simulation;
end AircraftSystem;
