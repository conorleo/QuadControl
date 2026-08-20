model Rotor
  parameter Real gainF = 1;
  parameter Real gainM = 1;
  
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque forceAndTorque(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b)  annotation(
    Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(origin = {98, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {98, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput thrust annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-92, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gainForce(k = gainF)  annotation(
    Placement(visible = true, transformation(origin = {-50, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gainMoment(k = gainM)  annotation(
    Placement(visible = true, transformation(origin = {-52, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant zero(k = 0)  annotation(
    Placement(visible = true, transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(forceAndTorque.frame_b, frame_a) annotation(
    Line(points = {{0, 0}, {98, 0}}, color = {95, 95, 95}));
  connect(gainForce.y, forceAndTorque.force[3]) annotation(
    Line(points = {{-38, 20}, {-32, 20}, {-32, 6}, {-22, 6}}, color = {0, 0, 127}));
  connect(gainMoment.y, forceAndTorque.torque[3]) annotation(
    Line(points = {{-40, -20}, {-32, -20}, {-32, -6}, {-22, -6}}, color = {0, 0, 127}));
  connect(gainForce.u, thrust) annotation(
    Line(points = {{-62, 20}, {-70, 20}, {-70, 0}, {-100, 0}}, color = {0, 0, 127}));
  connect(gainMoment.u, thrust) annotation(
    Line(points = {{-100, 0}, {-70, 0}, {-70, -20}, {-64, -20}}, color = {0, 0, 127}));
  connect(zero.y, forceAndTorque.force[1]) annotation(
    Line(points = {{-38, 70}, {-22, 70}, {-22, 6}}, color = {0, 0, 127}));
  connect(zero.y, forceAndTorque.force[2]) annotation(
    Line(points = {{-38, 70}, {-22, 70}, {-22, 6}}, color = {0, 0, 127}));
  connect(zero.y, forceAndTorque.torque[1]) annotation(
    Line(points = {{-38, 70}, {-22, 70}, {-22, -6}}, color = {0, 0, 127}));
  connect(zero.y, forceAndTorque.torque[2]) annotation(
    Line(points = {{-38, 70}, {-22, 70}, {-22, -6}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "4.0.0")));
end Rotor;
