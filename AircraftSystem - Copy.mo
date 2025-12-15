package AircraftSystem
  package Interfaces
    connector plane_environment_interfaces
      Integer phase;
      //1=Takeoff, 2=Cruising, 3=Landing"
      annotation(
        Icon(graphics = {Ellipse(fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-28, 28}, {28, -28}})}));
    end plane_environment_interfaces;

    connector mainForce
      flow Real Ft, Fl, Fd, Fw;
    end mainForce;

    connector energyConversion
    end energyConversion;
  end Interfaces;

  package Plane
    model Mass_model
      //A320neo
      parameter Real m = 41.000;
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
      Real alpha, v, Pitch_rate; // [Degree/Second]
      // Aircraft velocity [m/s]
      Real Lift, Drag;
      input Integer phase;
    equation
      
      der(alpha) = Pitch_rate;
      if (phase == 1) and (v < 80) then
        Pitch_rate = 0;
      elseif (phase == 1) and (v > 80) then
        Pitch_rate = 3;
      elseif phase == 2 then
        fuel_flow = -landing;
      else
        fuel_flow = 0;
      end if;
      Lift = 0.5*rho*v^2*S*CL;
// Lift formula
      Drag = 0.5*rho*v^2*S*CD;
// Drag formula
    end Aerodynamics;

    model Propulsion
      parameter Real Tmax = 280000; // [N]
      // Maximum thrust [N]
      Real Throttle;
      // 0..1
      Real Thrust;
      Real Weight = 60000; // [example weight]
      Real A, V;
      input Integer phase;
      
    equation
      Thrust = Throttle*Tmax;
      A = Thrust/Weight;
      der(V) = A;
      if phase == 1 then
        Throttle = 1;
      elseif phase == 2 then
        Throttle = 0.2;
      elseif phase == 3 then
        Throttle = 0.1;
      else
        A = 0;
      end if;
    end Propulsion;

    model Fuel_tank
      parameter Real Max_fuel_capacity = 29.6;
      //[m^3]
      parameter Real Max_fuel_mass = 237000;
      //[kg]
      parameter Real Fuel_density = 800;
      //[kg/m^3]
      parameter Real takeoff = 0.35;
      //[kg/s] one engine
      parameter Real cruising = 0.25;
      parameter Real landing = 0.08;
      input Integer phase;
      Real Fuel_mass(start = 0);
      Real fuel_flow;
    equation
      der(Fuel_mass) = fuel_flow;
      if phase == 1 then
        fuel_flow = -takeoff;
      elseif phase == 2 then
        fuel_flow = -cruising;
      elseif phase == 3 then
        fuel_flow = -landing;
      else
        fuel_flow = 0;
      end if;
      annotation(
        Icon,
        Diagram);
    end Fuel_tank;

    model Engine
    equation

    end Engine;

    model Capacity
      parameter Real max_passenger_cap = 195;
      //[People]
      parameter Real passenger_avg_weight = 65;
      //[kg]
      parameter Real max_cargo = 5000;
      //[kg]
      input Real passenger, cargo;
    equation

    end Capacity;

    model Airplane_model
      // Submodel instantiation
      Aerodynamics aero;
      Propulsion prop;
      Capacity cap;
      Fuel_tank fuel(Fuel_mass(start = 220000));
      
    Interfaces.plane_environment_interfaces plane_environment_interfaces1 annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, -2}, extent = {{-10, -10}, {10, 10}})));
    equation
      aero.v = 800;
      prop.Throttle = 0.8;
      fuel.phase = plane_environment_interfaces1.phase;
      aero.phase = plane_environment_interfaces1.phase;
      cap.passenger = 150;
      cap.cargo = 2000;
    
      annotation(
        Icon(graphics = {Polygon(origin = {-12, 1}, fillColor = {255, 255, 0}, fillPattern = FillPattern.Solid, points = {{-56, 1}, {2, 3}, {30, 23}, {18, 3}, {60, -1}, {68, 5}, {64, -5}, {22, -9}, {48, -23}, {4, -9}, {-68, -3}, {-58, 1}, {-56, 1}})}));
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

    model Enviroment_model
    equation

    end Enviroment_model;
  end Environment;

  package Flight_Phases
    model Takeoff
  Interfaces.plane_environment_interfaces plane_environment_interfaces1 annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}), iconTransformation(extent = {{-10, -10}, {10, 10}})));
    equation
      plane_environment_interfaces1.phase = 1;
      annotation(
        Icon(graphics = {Rectangle(fillColor = {170, 170, 127}, fillPattern = FillPattern.Forward, extent = {{-60, 10}, {60, -10}}), Line(origin = {-45.8818, 18.8818}, points = {{11.8818, -12.8818}, {-12.1182, 13.1182}, {-2.11819, 9.11819}, {-10.1182, 3.11819}, {-12.1182, 13.1182}})}));
    end Takeoff;

    model Cruising
  Interfaces.plane_environment_interfaces plane_environment_interfaces1 annotation(
        Placement(transformation(origin = {0, 22}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 22}, extent = {{-10, -10}, {10, 10}})));
    equation
      plane_environment_interfaces1.phase = 2;
      annotation(
        Icon(graphics = {Rectangle(origin = {0, 29}, fillColor = {85, 170, 255}, fillPattern = FillPattern.Solid, extent = {{-62, 29}, {62, -29}}), Rectangle(origin = {0, 49}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-62, 9}, {62, -9}})}));
    end Cruising;

    model Landing
  Interfaces.plane_environment_interfaces plane_environment_interfaces1 annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}), iconTransformation(extent = {{-10, -10}, {10, 10}})));
    equation
      plane_environment_interfaces1.phase = 3;
      annotation(
        Diagram(graphics),
        Icon(graphics = {Rectangle(fillColor = {170, 170, 255}, fillPattern = FillPattern.Backward, extent = {{-60, 10}, {60, -10}}), Line(origin = {58.1182, 16.8818}, rotation = 90, points = {{11.8818, -12.8818}, {-12.1182, 13.1182}, {-2.11819, 9.11819}, {-10.1182, 3.11819}, {-12.1182, 13.1182}})}));
    end Landing;
  end Flight_Phases;

  package Simulation
    model Ex_takeoff
  Flight_Phases.Takeoff takeoff annotation(
        Placement(transformation(origin = {0, -22}, extent = {{-38, -38}, {38, 38}})));
  Plane.Airplane_model airplane_model annotation(
        Placement(transformation(origin = {-35, 1}, extent = {{-33, -33}, {33, 33}})));
    equation
  connect(airplane_model.plane_environment_interfaces1, takeoff.plane_environment_interfaces1) annotation(
        Line(points = {{-34, 0}, {0, 0}, {0, -22}}));
    end Ex_takeoff;

    model Ex_cruising
  Flight_Phases.Cruising cruising annotation(
        Placement(transformation(origin = {0, -12}, extent = {{-56, -56}, {56, 56}})));
  Plane.Airplane_model airplane_model annotation(
        Placement(transformation(origin = {3, 11}, extent = {{-35, -35}, {35, 35}})));
    equation
  connect(airplane_model.plane_environment_interfaces1, cruising.plane_environment_interfaces1) annotation(
        Line(points = {{4, 10}, {0, 10}, {0, 0}}));
    end Ex_cruising;

    model Ex_landing
  Flight_Phases.Landing landing annotation(
        Placement(transformation(origin = {0, -22}, extent = {{-42, -42}, {42, 42}})));
  Plane.Airplane_model airplane_model annotation(
        Placement(transformation(origin = {56, 0}, extent = {{-38, -38}, {38, 38}})));
    equation
  connect(airplane_model.plane_environment_interfaces1, landing.plane_environment_interfaces1) annotation(
        Line(points = {{56, 0}, {0, 0}, {0, -22}}));
    end Ex_landing;
  end Simulation;
end AircraftSystem;
