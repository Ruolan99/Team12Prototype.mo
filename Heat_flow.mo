package Heat_flow

  model Engine
    parameter Real fuel_rate = 1 "kg/s";
    parameter Real fuel_heat = 43e6;
    parameter Real T_max = 10000000 "N";
    
    Modelica.Blocks.Interfaces.RealInput Throttle annotation(
      Placement(transformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealOutput Heat annotation(
      Placement(transformation(origin = {110, -4}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
  equation
    Heat = Throttle*fuel_rate*fuel_heat;
    annotation(
      Icon(graphics = {Rectangle(origin = {3, -3}, extent = {{-17, 15}, {17, -15}})}));
  end Engine;

  model EngineSystem
    Real total_Q; 
    Real thro(start = 0);
    parameter Real target_Q = 40e6;
  Modelica.Blocks.Interfaces.RealInput Q1 annotation(
      Placement(transformation(origin = {-94, -48}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-109, 9}, extent = {{-9, -9}, {9, 9}})));
  Modelica.Blocks.Interfaces.RealInput Q2 annotation(
      Placement(transformation(origin = {-92, -76}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-109, 29}, extent = {{-9, -9}, {9, 9}})));
  Modelica.Blocks.Interfaces.RealInput Q3 annotation(
      Placement(transformation(origin = {-88, -92}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-109, -11}, extent = {{-9, -9}, {9, 9}})));
  Modelica.Blocks.Interfaces.RealInput Q4 annotation(
      Placement(transformation(origin = {-86, -98}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-109, -31}, extent = {{-9, -9}, {9, 9}})));
  Modelica.Blocks.Interfaces.RealOutput Throttle annotation(
      Placement(transformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
  equation
    total_Q = Q1 + Q2 + Q3 + Q4;
    thro = max(0, min(1, target_Q / total_Q));
    Throttle = thro;
  end EngineSystem;

  model test
  EngineSystem engineSystem1 annotation(
      Placement(transformation(origin = {22, -10}, extent = {{-28, -28}, {28, 28}})));
  Engine engine annotation(
      Placement(transformation(origin = {-57, 35}, extent = {{-17, -17}, {17, 17}})));
  Modelica.Blocks.Sources.Constant zero(k = 0)  annotation(
      Placement(transformation(origin = {-54, -36}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant zero1(k = 0) annotation(
      Placement(transformation(origin = {-54, -14}, extent = {{-10, -10}, {10, 10}})));
  Heat_flow.Engine engine1 annotation(
      Placement(transformation(origin = {-57, 13}, extent = {{-17, -17}, {17, 17}})));
  equation
    connect(engine.Heat, engineSystem1.Q2) annotation(
      Line(points = {{-38, 36}, {-8, 36}, {-8, -2}}, color = {0, 0, 127}));
    connect(zero.y, engineSystem1.Q4) annotation(
      Line(points = {{-42, -36}, {-22, -36}, {-22, -18}, {-8, -18}}, color = {0, 0, 127}));
    connect(zero1.y, engineSystem1.Q3) annotation(
      Line(points = {{-42, -14}, {-8, -14}}, color = {0, 0, 127}));
  connect(engine1.Heat, engineSystem1.Q1) annotation(
      Line(points = {{-38, 14}, {-18, 14}, {-18, -8}, {-8, -8}}, color = {0, 0, 127}));
  connect(engineSystem1.Throttle, engine.Throttle) annotation(
      Line(points = {{52, -10}, {64, -10}, {64, 54}, {-88, 54}, {-88, 36}, {-78, 36}}, color = {0, 0, 127}));
  connect(engineSystem1.Throttle, engine1.Throttle) annotation(
      Line(points = {{52, -10}, {64, -10}, {64, 54}, {-88, 54}, {-88, 14}, {-78, 14}}, color = {0, 0, 127}));
    annotation(
      Diagram);
end test;

annotation(
    uses(Modelica(version = "4.1.0")));
end Heat_flow;
