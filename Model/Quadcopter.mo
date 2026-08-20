model Quadcopter
  parameter Real m = 1;     // mass, m
  parameter Real l = 1;     // spar length, m
  parameter Real gainF = 1; // rotor force gain magnitude (pre-multiplies input decimal in range 0-1)
  parameter Real gainM = 1; // rotor torque gain

  Modelica.Mechanics.MultiBody.Parts.Body body(enforceStates = true, m = m)  annotation(
    Placement(visible = true, transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation spar1(r = {0, l, 0})  annotation(
    Placement(visible = true, transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation spar2(r = {l, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation spar3(r = {0, -l, 0})  annotation(
    Placement(visible = true, transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation spar4(r = {-l, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {-30, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Rotor rotor1(gainF = gainF, gainM = gainM)  annotation(
    Placement(visible = true, transformation(origin = {0, 70}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
  Rotor rotor2(gainF = gainF, gainM = -gainM)  annotation(
    Placement(visible = true, transformation(origin = {70, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Rotor rotor3(gainF = gainF, gainM = gainM)  annotation(
    Placement(visible = true, transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Rotor rotor4(gainF = gainF, gainM = -gainM)  annotation(
    Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput thrust1 annotation(
    Placement(visible = true, transformation(origin = {0, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-10, 92}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput thrust2 annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{20, -20}, {-20, 20}}, rotation = 0), iconTransformation(origin = {110, 8}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput thrust3 annotation(
    Placement(visible = true, transformation(origin = {0, -110}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {-18, -54}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput thrust4 annotation(
    Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(spar1.frame_a, body.frame_a) annotation(
    Line(points = {{0, 20}, {0, 0}}));
  connect(spar2.frame_a, body.frame_a) annotation(
    Line(points = {{0, 0}, {20, 0}}, color = {95, 95, 95}));
  connect(spar3.frame_a, body.frame_a) annotation(
    Line(points = {{0, 0}, {0, -20}}, color = {95, 95, 95}));
  connect(spar4.frame_a, body.frame_a) annotation(
    Line(points = {{-20, 0}, {0, 0}}));
  connect(spar1.frame_b, rotor1.frame_a) annotation(
    Line(points = {{0, 40}, {0, 60}}));
  connect(spar2.frame_b, rotor2.frame_a) annotation(
    Line(points = {{40, 0}, {60, 0}}, color = {95, 95, 95}));
  connect(spar3.frame_b, rotor3.frame_a) annotation(
    Line(points = {{0, -40}, {0, -60}}));
  connect(spar4.frame_b, rotor4.frame_a) annotation(
    Line(points = {{-40, 0}, {-60, 0}}));
  connect(thrust1, rotor1.thrust) annotation(
    Line(points = {{0, 110}, {0, 80}}, color = {0, 0, 127}));
  connect(thrust2, rotor2.thrust) annotation(
    Line(points = {{110, 0}, {80, 0}}, color = {0, 0, 127}));
  connect(thrust3, rotor3.thrust) annotation(
    Line(points = {{0, -110}, {0, -80}}, color = {0, 0, 127}));
  connect(thrust4, rotor4.thrust) annotation(
    Line(points = {{-110, 0}, {-80, 0}}, color = {0, 0, 127}));

annotation(
    uses(Modelica(version = "4.0.0")));
end Quadcopter;
