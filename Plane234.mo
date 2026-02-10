package Plane234
  package Interfaces
    connector Kin
      Real thr;
      flow Real Ft;
      Real gamma;
      flow Real Fl;
      Real h, alpha, h_c, x;
      Real m_e, m, v, g, rho, m_lost;
      discrete Integer phase;
      annotation(
        Icon(graphics = {Ellipse(fillPattern = FillPattern.Solid, extent = {{-66, 64}, {66, -64}})}));
    end Kin;
  end Interfaces;

  package Aero
    model Wing
      parameter Real s = 122.6;
      parameter Real Cl0 = 0.6;
      parameter Real Cl_alpha = 4.39;
      parameter Real Cd0 = 0.022;
      parameter Real k = 0.045;
      Real rho, v = kin.v, alpha = kin.alpha, L, D, h = kin.h, Cl, Cd;
      Interfaces.Kin kin annotation(
        Placement(transformation(origin = {-18, 2}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-18, 2}, extent = {{-10, -10}, {10, 10}})));
    equation
      rho = 1.225*exp(-h/8500);
      Cl = Cl0 + Cl_alpha*alpha;
      Cd = Cd0 + k*Cl^2;
      L = 0.5*rho*v^2*s*Cl;
      D = 0.5*rho*v^2*s*Cd;
      kin.Fl = L;
      kin.Ft = -D;
      annotation(
        Icon(graphics = {Polygon(origin = {0, 2}, fillColor = {255, 255, 0}, fillPattern = FillPattern.Solid, points = {{-38, 2}, {38, 62}, {36, -62}, {38, -60}, {-38, 2}}), Text(origin = {10, 3}, extent = {{15, -9}, {-15, 9}}, textString = "Wing")}));
    end Wing;
  end Aero;

  package Body
    connector People
      Real rate;
      flow Real m_people;
      annotation(
        Icon(graphics = {Ellipse(origin = {-14, 40}, extent = {{-10, 10}, {10, -10}}), Rectangle(origin = {-13, 5}, extent = {{-9, 23}, {9, -23}}), Rectangle(origin = {12, -12}, extent = {{-16, 6}, {16, -6}}), Rectangle(origin = {22, -34}, extent = {{-6, 16}, {6, -16}})}));
    end People;

    connector Fuel
      Real rate;
      flow Real m_tank;
      annotation(
        Icon(graphics = {Ellipse(origin = {3, -2},lineColor = {255, 170, 0}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Solid, extent = {{-85, 86}, {85, -86}}), Text(origin = {0, 3}, extent = {{-36, 33}, {36, -33}}, textString = "Fuel")}));
    end Fuel;

    model People_mass
      People people annotation(
        Placement(transformation(origin = {2, 6}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-72, 68}, extent = {{-10, -10}, {10, 10}})));
      parameter Real m_people = 2800;
      //For 40 people
    equation
      people.m_people = -m_people;
      annotation(
        Icon(graphics = {Rectangle(origin = {0, 2}, extent = {{-96, 92}, {96, -92}}), Text(origin = {-54, 19}, extent = {{-24, 17}, {24, -17}}, textString = "40"), Text(origin = {-17, -27}, extent = {{-55, 31}, {55, -31}}, textString = "Passengers")}),
  Diagram(graphics));
    end People_mass;

    model Fuel_mass
      Fuel fuel annotation(
        Placement(transformation(origin = {2, -14}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-73, 65}, extent = {{-11, -11}, {11, 11}})));
      parameter Real m_tank = 5000;
    equation
      fuel.m_tank = -m_tank;
      annotation(
        Icon(graphics = {Rectangle(origin = {1, -1}, extent = {{-97, 91}, {97, -89}}), Text(origin = {2, 3}, extent = {{-80, 25}, {80, -25}}, textString = "Fuel tank")}),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
  Diagram(graphics));
    end Fuel_mass;

    model Mass
      Real g = kin.g;
      parameter Real m_mono = 50000;
      parameter Real rate = 1;
      Real m_tank;
      Real m_fuel(start = 0);
      Real m_people;
      Real m_lost;
      Real thr = kin.thr;
      Real m_w;
      Real m;
      Real W;
      Real m_e = kin.m_e;
      Real gamma = kin.gamma;
      People people annotation(
        Placement(transformation(origin = {50, -2}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {49, -27}, extent = {{-21, -21}, {21, 21}})));
      Fuel fuel annotation(
        Placement(transformation(origin = {-52, -22}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-42, -32}, extent = {{-20, -20}, {20, 20}})));
      Interfaces.Kin kin annotation(
        Placement(transformation(origin = {-8, 44}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-8, 44}, extent = {{-10, -10}, {10, 10}})));
    equation
      W = m*g;
      m = m_w + m_e;
      m_w = m_people + m_mono + max(0, m_lost);
      der(m_fuel) = -thr*rate;
      m_lost = m_tank + m_fuel;
      people.rate = rate;
      fuel.rate = rate;
      m_people = people.m_people;
      m_tank = fuel.m_tank;
      kin.Fl = -W*cos(gamma);
      kin.Ft = -W*sin(gamma);
      kin.m = m;
      kin.m_lost = m_lost;
      
      annotation(
        Icon(graphics = {Rectangle(origin = {1, 2}, fillColor = {255, 170, 255}, fillPattern = FillPattern.Backward, extent = {{-73, 66}, {73, -66}})}));
    end Mass;
  end Body;

  package Env
    model Ground
      parameter Real kb = 0.5;
      parameter Real kn = 2000000;
      parameter Real c = 100000;
      Real N, B, m = kin.m, v = kin.v, h = kin.h, h_c = kin.h_c;
      discrete Integer phase = kin.phase;
      Interfaces.Kin kin annotation(
        Placement(transformation(origin = {0, -20}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, -20}, extent = {{-10, -10}, {10, 10}})));
    
    equation
      
      B = if phase == 6 or phase == 7 then kb*(m*v^2)/2000 else 0;
      N = if phase == 1 or phase == 6 or phase == 7 then kn*(h_c - h) + c*der(h) else 0;
      kin.Fl = -B;
      kin.Ft = N;
      annotation(
        Icon(graphics = {Line(origin = {0, -20}, points = {{-80, 0}, {80, 0}, {80, 0}}), Rectangle(origin = {0, -20}, fillColor = {85, 85, 0}, fillPattern = FillPattern.Horizontal, extent = {{-90, 8}, {90, -8}}), Text(origin = {-46, -20}, textColor = {255, 255, 255}, extent = {{-24, 4}, {24, -4}}, textString = "Ground")}));
    end Ground;

    model Atm
      parameter Real g = 9.80665; 
// Gravitational acceleration [m/s^2]
      parameter Real T0 = 288.15; 
// Sea-level standard temperature [K]
      parameter Real p0 = 101325; 
// Sea-level standard pressure [Pa]
      parameter Real L = 0.0065; 
// Temperature lapse rate in the troposphere [K/m]
      parameter Real R = 287.0; 
// Specific gas constant for dry air [J/(kg·K)]
      Real rho; 
      Real T; 
      Real p; 
      Real h = kin.h;
      
      Interfaces.Kin kin annotation(
        Placement(transformation(origin = {-2, -2}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-2, -2}, extent = {{-10, -10}, {10, 10}})));
      
    algorithm
    
// ===== ISA temperature model =====
      T := T0 - L*h; // ===== ISA pressure model =====
      p := p0*(T/T0)^(g/(L*R)); // ===== Air density calculation =====
      rho := p/(R*T);
    equation
      kin.rho = rho;
      kin.Fl = 0;
      kin.Ft = 0;
      kin.g = g;
    
      annotation(
        Icon(graphics = {Rectangle(origin = {1, -1}, fillColor = {170, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-91, 53}, {91, -53}}), Text(origin = {0, -24}, textColor = {255, 255, 255}, extent = {{-38, 8}, {38, -8}}, textString = "Atmosphere")}));
    end Atm;
  end Env;

  package Noz
    connector EngCon
      Real e_v;
      flow Real Q;
      annotation(
        Icon(graphics = {Ellipse(origin = {1, 0}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid, extent = {{-27, 26}, {27, -26}})}));
    end EngCon;

    model Nozzle
      EngCon engCon annotation(
        Placement(transformation(origin = {0, 2}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 2}, extent = {{-10, -10}, {10, 10}})));
      parameter Real Q = 43e6;
    equation
      engCon.Q = Q;
      annotation(
        Icon(graphics = {Line(origin = {14.0374, 1}, points = {{29.9626, 39}, {-58.0374, -41}, {-56.0374, 39}, {29.9626, -39}, {29.9626, 39}, {29.9626, 41}}), Text(origin = {0, -42}, extent = {{-20, 22}, {20, -22}}, textString = "Nozzle")}));
    end Nozzle;

    model Eng
      parameter Real fuel_rate = 1 "kg/s";
      parameter Real LHV = 40e6;
      parameter Real eff = 0.8;
      parameter Real e_v = 300;
      parameter Real m_n = 2000;
      Real T, m_e, thr = kin.thr, alpha = kin.alpha;
      EngCon engCon annotation(
        Placement(transformation(origin = {-12, -20}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {12, 0}, extent = {{-10, -10}, {10, 10}})));
      Interfaces.Kin kin annotation(
        Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
    equation
      engCon.e_v = e_v;
      T = -thr*engCon.Q*eff/e_v;
      m_e = engCon.Q/LHV*m_n;
      kin.m_e = -m_e;
      kin.Fl = T*sin(alpha);
      kin.Ft = T*cos(alpha);
      annotation(
        Icon(graphics = {Rectangle(origin = {-2, -4}, fillColor = {255, 85, 0}, fillPattern = FillPattern.Vertical, extent = {{-66, 58}, {66, -58}}), Text(origin = {-6, -32}, extent = {{-36, 14}, {36, -14}}, textString = "Engine")}));
    end Eng;
  end Noz;

  package Control
    model Control
      // States
      Real thr(start = 0.0);
      Real theta(start = 0.05);
      Real alpha(start = 0.0);
      Real gamma(start = 0.0);
      Real h = kin.h;
      Real x = kin.x;
      Real v = kin.v;
      Real m_lost = kin.m_lost;
    // Phase tracker (discrete to prevent chattering)
      discrete Integer phase(start = 1);
      // Constants
      parameter Real kp = 1;
      parameter Real kv = 1;
      parameter Real kb = 0.5;
      // Actuator dynamics parameters
      parameter Real alpha_rate_limit = 0.2;
      // Altitude hold parameters
      parameter Real kp_h = 0.001;
      parameter Real kd_h = 0.0005;
      parameter Real gamma_max = 0.35;
      parameter Real gamma_min = -0.35;
      // Variables
      parameter Real x_t = 1000000;
      parameter Real h_t = 11000;
      parameter Real v_t = 230;
      parameter Real gamma_t = 0.262;
      Real gamma_c, gamma_auto, v_c, h_c;
      Real alpha_cmd;
      Real thr_cmd;
      Real alpha_rate;
      Boolean altitude_hold;
      Interfaces.Kin kin annotation(
        Placement(transformation(origin = {-4, -10}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-4, -10}, extent = {{-10, -10}, {10, 10}})));
    algorithm
      when v > 0.35*v_t and phase == 1 then
        phase := 2;
      elsewhen h > 0.9*h_t and phase == 2 then
        phase := 3;
      elsewhen (x_t - x) < 6000 and phase == 3 then
        phase := 4;
      elsewhen h <= 0.05*h_t and h > 0 and phase == 4 then
        phase := 5;
      elsewhen h <= 0 and phase == 5 then
        phase := 6;
      elsewhen h <= 0 and v < 0.22*v_t and phase == 6 then
        phase := 7;
      end when;
    equation
// Compute automatic gamma from altitude hold (PD controller, P responds to altitude error, D damps the response)
      gamma_auto = max(gamma_min, min(gamma_max, kp_h*(h_c - h) - kd_h*der(h)));
// Choose between altitude hold and manual gamma (P controller)
      alpha_cmd = if altitude_hold then kp*(gamma_auto - gamma) else kp*(gamma_c - gamma);
      thr_cmd = kv*(v_c - v);
      theta = alpha + gamma;
//first-oder alpha
      alpha_rate = (min(0.349, max(0, alpha_cmd)) - alpha);
      der(alpha) = max(-alpha_rate_limit, min(alpha_rate_limit, alpha_rate));
      thr = if m_lost > 0 then min(1, max(0, thr_cmd)) else 0;
//Feedback
      kin.gamma = gamma;
      kin.alpha = alpha;
      kin.h_c = h_c;
      kin.thr = thr;
      kin.phase = phase;
      kin.Fl = 0;
      kin.Ft = 0;
// Phase-based targets
      if phase == 1 then
        v_c = 0.6*v_t;
        gamma_c = 0*gamma_t;
        h_c = 0*h_t;
        altitude_hold = false;
      elseif phase == 2 then
        v_c = 1*v_t;
        gamma_c = 1*gamma_t;
        h_c = 1*h_t;
        altitude_hold = true;
      elseif phase == 3 then
        v_c = 1*v_t;
        gamma_c = 0*gamma_t;
        h_c = 1*h_t;
        altitude_hold = true;
      elseif phase == 4 then
        v_c = 0.43*v_t;
        gamma_c = -0.7*gamma_t;
        h_c = 0.05*h_t;
        altitude_hold = false;
      elseif phase == 5 then
        v_c = 0*v_t;
        gamma_c = 0.075*gamma_t;
        h_c = 0*h_t;
        altitude_hold = false;
      else
        v_c = 0;
        gamma_c = 0;
        h_c = 0;
        altitude_hold = false;
      end if;
      annotation(
        Icon(graphics = {Ellipse(origin = {-47, -10}, fillColor = {85, 170, 127}, fillPattern = FillPattern.Solid, extent = {{-9, 12}, {9, -12}}), Ellipse(origin = {41, -11}, fillColor = {85, 170, 127}, fillPattern = FillPattern.Solid, extent = {{-11, 13}, {11, -13}}), Rectangle(origin = {-3, -11}, fillColor = {85, 170, 127}, fillPattern = FillPattern.Solid, extent = {{-43, 5}, {43, -5}})}));
    end Control;
  end Control;

  package Dyna
    model Kinematics
      Real h(start = 0);
      Real x(start = 0);
      Real v(start = 0.01);
      Real thr = kin.thr;
      Real alpha = kin.alpha;
      Real gamma = kin.gamma;
      Real m = kin.m;
      Real h_c = kin.h_c;
      Real g = kin.g;
      Real Ft, Fl;
      Real m_lost = kin.m_lost;
      discrete Integer phase = kin.phase;
      Interfaces.Kin kin annotation(
        Placement(transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}})));
    equation
// Kinematics
      der(h) = if h > 0 or v*sin(gamma) > 0 then v*sin(gamma) else 0;
      der(x) = v*cos(gamma);
// Force balance
      Ft = -kin.Ft;
      Fl = -kin.Fl;
      if phase == 7 then
        der(gamma) = -gamma/10;
        der(v) = -v/100;
      else
        Fl = m*v*der(gamma);
        Ft = m*der(v);
      end if;
      kin.h = h;
      kin.x = x;
      kin.v = v;
      annotation(
        Icon(graphics = {Polygon(origin = {3, 13}, fillColor = {255, 170, 127}, fillPattern = FillPattern.Solid, points = {{-59, 35}, {-85, -39}, {3, -65}, {85, -3}, {29, 55}, {-13, 65}, {-59, 35}})}));
    end Kinematics;
  end Dyna;

  package Example
    model example
      Dyna.Kinematics kinematics annotation(
        Placement(transformation(origin = {-49, 21}, extent = {{-21, -21}, {21, 21}})));
      Control.Control control annotation(
        Placement(transformation(origin = {-26, 84}, extent = {{-30, -30}, {30, 30}})));
      Noz.Eng eng annotation(
        Placement(transformation(origin = {24, 22}, extent = {{-30, -30}, {30, 30}})));
      Noz.Nozzle nozzle annotation(
        Placement(transformation(origin = {33, 55}, extent = {{-33, -33}, {33, 33}})));
      Noz.Nozzle nozzle1 annotation(
        Placement(transformation(origin = {33, -15}, extent = {{-33, -33}, {33, 33}})));
      Env.Atm atm annotation(
        Placement(transformation(origin = {-81, 55}, extent = {{-15, -15}, {15, 15}})));
      Env.Ground ground annotation(
        Placement(transformation(origin = {-72, -2}, extent = {{-26, -26}, {26, 26}})));
      Body.Mass mass annotation(
        Placement(transformation(origin = {-28, -2}, extent = {{-16, -16}, {16, 16}})));
      Body.People_mass people_mass annotation(
        Placement(transformation(origin = {-1, -63}, extent = {{-15, -15}, {15, 15}})));
      Body.People_mass people_mass1 annotation(
        Placement(transformation(origin = {-1, -31}, extent = {{-15, -15}, {15, 15}})));
      Body.Fuel_mass fuel_mass annotation(
        Placement(transformation(origin = {-65, -61}, extent = {{-15, -15}, {15, 15}})));
      Body.Fuel_mass fuel_mass1 annotation(
        Placement(transformation(origin = {-65, -29}, extent = {{-15, -15}, {15, 15}})));
      Aero.Wing wing annotation(
        Placement(transformation(origin = {-14, 22}, extent = {{-36, -36}, {36, 36}})));
    equation
      connect(wing.kin, kinematics.kin) annotation(
        Line(points = {{-20, 23}, {-49, 23}}));
      connect(control.kin, kinematics.kin) annotation(
        Line(points = {{-27, 81}, {-27, 42}, {-49, 42}, {-49, 23}}));
      connect(eng.kin, kinematics.kin) annotation(
        Line(points = {{9, 22}, {-20.5, 22}, {-20.5, 23}, {-49, 23}}));
      connect(nozzle.engCon, eng.engCon) annotation(
        Line(points = {{33, 56}, {33, 22}, {28, 22}}));
      connect(nozzle1.engCon, eng.engCon) annotation(
        Line(points = {{33, -14}, {33, 22}, {28, 22}}));
      connect(atm.kin, kinematics.kin) annotation(
        Line(points = {{-81, 55}, {-81, 23}, {-49, 23}}));
      connect(mass.kin, kinematics.kin) annotation(
        Line(points = {{-29, 5}, {-29, 23}, {-49, 23}}));
      connect(fuel_mass.fuel, mass.fuel) annotation(
        Line(points = {{-76, -51}, {-76, -57.5}, {-35, -57.5}, {-35, -7}}));
      connect(fuel_mass1.fuel, mass.fuel) annotation(
        Line(points = {{-76, -19}, {-76, -7}, {-35, -7}}));
  connect(ground.kin, kinematics.kin) annotation(
        Line(points = {{-72, -7}, {-72, 24}, {-48, 24}}));
  connect(mass.people, people_mass1.people) annotation(
        Line(points = {{-20, -6}, {-20, -21}, {-12, -21}}));
  connect(mass.people, people_mass.people) annotation(
        Line(points = {{-20, -6}, {-20, -53}, {-12, -53}}));
    annotation(
        experiment(StartTime = 0, StopTime = 8000, Tolerance = 1e-06, Interval = 16));
end example;
  end Example;
end Plane234;
