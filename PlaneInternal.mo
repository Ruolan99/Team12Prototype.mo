package PlaneInternal

  connector FuelPort
    Real p             "Pressure (across)";
    flow Real m_flow   "Mass flow rate (flow)";
  end FuelPort;

  // Balanced connector: 1 potential (T) + 1 flow (Q_flow)
  connector HeatPort
    Real T             "Temperature (across)";
    flow Real Q_flow   "Heat flow rate (flow) [W]";
  end HeatPort;

  connector Flange
    Real s "Position (across)";
    flow Real f "Force (flow)";
  end Flange;

  model FuelSource
    FuelPort fuelOut;
    parameter Real m_fuel = 0.5 "kg/s (positive parameter)";
    parameter Real p0 = 101325 "Pa";
  equation
// Sign convention for 'flow' variables:
// Source must output negative to drive positive into connected component.
    fuelOut.m_flow = -m_fuel;
    fuelOut.p      = p0;
  end FuelSource;

  model Combustor
    FuelPort fuelIn;
    HeatPort heatOut;

    parameter Real LHV = 43e6 "J/kg";
  equation
// Produce heat flow from fuel flow
    heatOut.Q_flow = fuelIn.m_flow * LHV;

    // IMPORTANT: Do NOT prescribe heatOut.T here (would fight the sink/network).
    // heatOut.T is determined by whatever thermal boundary it's connected to.
  end Combustor;

  model Nozzle
    HeatPort heatIn;
    Flange flangeOut;
  
    parameter Real T_amb = 500;
    parameter Real efficiency = 0.8 "0..1";
    parameter Real v_e = 600 "m/s effective exhaust velocity";
    Real Q_waste;
  equation
// Waste stream shares the same temperature level (across variable equality)
    heatIn.T = T_amb;
    Q_waste = (1 - efficiency) * heatIn.Q_flow;
// Convert useful thermal power to thrust (numerically robust)
    flangeOut.f = (efficiency * heatIn.Q_flow) / v_e;
  end Nozzle;

  model Plane
    Flange flangeIn;
    parameter Real plane_mass = 10000;
    Real v(start=0, fixed=true);
    Real s(start=0, fixed=true);
  equation
    flangeIn.s = s;
  
    der(s) = v;
    plane_mass * der(v) = flangeIn.f;

  end Plane;

  model JetEngineSystem
    FuelSource fuel(m_fuel=0.5);
    Combustor combustor(LHV=43e6);
    Nozzle nozzle(efficiency=0.8, v_e=600);
    Plane plane;
  
  equation
    connect(fuel.fuelOut, combustor.fuelIn);
    connect(combustor.heatOut, nozzle.heatIn);
    connect(nozzle.flangeOut, plane.flangeIn);
  
  end JetEngineSystem;

end PlaneInternal;
