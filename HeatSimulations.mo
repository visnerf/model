package HeatSimulations
  model heatPumpAndStorage "Heat pump with scroll compressor connected to a simple room model with radiator"
    extends partialBaseCircuit;
    extends Modelica.Icons.Example;
    replaceable package MediumA = Buildings.Media.Air "Medium model for air";
    parameter Modelica.Units.SI.Volume V = 6*10*3 "Room volume";
    parameter Modelica.Units.SI.MassFlowRate mA_flow_nominal = V*1.2*6/3600 "Nominal mass flow rate";
    parameter Modelica.Units.SI.HeatFlowRate QRooInt_flow = 4000 "Internal heat gains of the room";
    //------------------------------------------------------------------------------//
    Buildings.Fluid.MixingVolumes.MixingVolume vol(redeclare package Medium = MediumA, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, m_flow_nominal = mA_flow_nominal, V = V) annotation(
      Placement(transformation(origin = {0, 26}, extent = {{60, 20}, {80, 40}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor theCon(G = 20000/40) "Thermal conductance with the ambient" annotation(
      Placement(transformation(origin = {0, 26}, extent = {{20, 40}, {40, 60}})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heaCap(C = 2*V*1.2*1006) "Heat capacity for furniture and walls" annotation(
      Placement(transformation(origin = {0, 26}, extent = {{60, 50}, {80, 70}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temRoo "Room temperature" annotation(
      Placement(transformation(origin = {-40, 56}, extent = {{10, -10}, {-10, 10}})));
    //----------------------------------------------------------------------------//
    //----------------------------------------------------------------------------//
    //------------------------------------------------------------------------------------//
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam = Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos")) "Weather data reader" annotation(
      Placement(transformation(origin = {-6, 26}, extent = {{-220, 40}, {-200, 60}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus "Weather data bus" annotation(
      Placement(transformation(origin = {0, 26}, extent = {{-160, 40}, {-140, 60}}), iconTransformation(extent = {{-160, 40}, {-140, 60}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature TOut "Outside temperature" annotation(
      Placement(transformation(origin = {0, 26}, extent = {{-20, 40}, {0, 60}})));
    //--------------------------------------------------------------------------------------//
    Modelica.Blocks.Logical.Hysteresis hysteresis(uLow = 273.15 + 19, uHigh = 273.15 + 21) "Hysteresis controller" annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {-190, -40})));
    Modelica.Blocks.Logical.Not not2 "Negate output of hysteresis" annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {-150, -40})));
    Modelica.Blocks.Math.BooleanToReal booToReaPum(realTrue = 1, y(start = 0)) "Pump signal" annotation(
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 180, origin = {-110, -110})));
    Modelica.Blocks.Logical.And and1 annotation(
      Placement(transformation(extent = {{-20, -90}, {0, -70}})));
    Modelica.Blocks.Logical.And and2 annotation(
      Placement(transformation(extent = {{20, -50}, {40, -30}})));
    Modelica.Blocks.Math.BooleanToReal booToReaPum1(realTrue = 273 + 45, y(start = 0)) "Pump signal" annotation(
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 90, origin = {40, -110})));
    Modelica.Blocks.Logical.Hysteresis tesConHea(uHigh = 0.25*mHeaPum_flow_nominal, uLow = 0.20*mHeaPum_flow_nominal) "Test for flow rate of condenser pump" annotation(
      Placement(transformation(extent = {{10, 10}, {-10, -10}}, rotation = 180, origin = {-50, -80})));
    Modelica.Blocks.Logical.Hysteresis tesEvaPum(uLow = 0.20*mHeaPum_flow_nominal, uHigh = 0.25*mHeaPum_flow_nominal) "Test for flow rate of evaporator pump" annotation(
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = -90, origin = {-30, -110})));
    Buildings.Fluid.Sources.Boundary_pT preSou(redeclare package Medium = MediumW, T = TRadSup_nominal, nPorts = 2) "Source for pressure and to account for thermal expansion of water" annotation(
      Placement(transformation(extent = {{90, -130}, {70, -110}})));
    replaceable Buildings.Fluid.Sources.Boundary_pT bou(redeclare package Medium = MediumW, nPorts = 1) constrainedby Buildings.Fluid.Sources.BaseClasses.PartialSource annotation(
       Placement(transformation(origin = {136, -6}, extent = {{10, -10}, {-10, 10}})));
  equation
    connect(theCon.port_b, vol.heatPort) annotation(
      Line(points = {{40, 76}, {50, 76}, {50, 56}, {60, 56}}, color = {191, 0, 0}));
    connect(heaCap.port, vol.heatPort) annotation(
      Line(points = {{70, 76}, {50, 76}, {50, 56}, {60, 56}}, color = {191, 0, 0}));
    connect(temRoo.port, vol.heatPort) annotation(
      Line(points = {{-30, 56}, {60, 56}}, color = {191, 0, 0}));
    connect(rad.heatPortCon, vol.heatPort) annotation(
      Line(points = {{34, 31.2}, {34, 56}, {60, 56}}, color = {191, 0, 0}));
    connect(rad.heatPortRad, vol.heatPort) annotation(
      Line(points = {{38, 31.2}, {38, 56}, {60, 56}}, color = {191, 0, 0}));
    connect(weaDat.weaBus, weaBus) annotation(
      Line(points = {{-206, 76}, {-150, 76}}, color = {255, 204, 51}, thickness = 0.5));
    connect(weaBus.TDryBul, TOut.T) annotation(
      Line(points = {{-149.95, 76.05}, {-86, 76.05}, {-86, 76}, {-22, 76}}, color = {255, 204, 51}, thickness = 0.5));
    connect(TOut.port, theCon.port_a) annotation(
      Line(points = {{0, 76}, {20, 76}}, color = {191, 0, 0}));
    connect(hysteresis.y, not2.u) annotation(
      Line(points = {{-179, -40}, {-162, -40}}, color = {255, 0, 255}));
    connect(temRoo.T, hysteresis.u) annotation(
      Line(points = {{-51, 56}, {-220, 56}, {-220, -40}, {-202, -40}}, color = {0, 0, 127}));
    connect(tesConHea.y, and1.u1) annotation(
      Line(points = {{-39, -80}, {-22, -80}}, color = {255, 0, 255}));
    connect(tesEvaPum.y, and1.u2) annotation(
      Line(points = {{-30, -99}, {-30, -88}, {-22, -88}}, color = {255, 0, 255}));
    connect(booToReaPum.y, pumHeaPum.m_flow_in) annotation(
      Line(points = {{-99, -110}, {-82, -110}}, color = {0, 0, 127}));
    connect(booToReaPum.y, pumHeaPumSou.m_flow_in) annotation(
      Line(points = {{-99, -110}, {-90, -110}, {-90, -180}, {-42, -180}}, color = {0, 0, 127}));
    connect(not2.y, and2.u1) annotation(
      Line(points = {{-139, -40}, {18, -40}}, color = {255, 0, 255}));
    connect(not2.y, booToReaPum.u) annotation(
      Line(points = {{-139, -40}, {-130, -40}, {-130, -110}, {-122, -110}}, color = {255, 0, 255}));
    connect(pumHeaPum.m_flow_actual, tesConHea.u) annotation(
      Line(points = {{-75, -99}, {-75, -80}, {-62, -80}}, color = {0, 0, 127}));
    connect(and1.y, and2.u2) annotation(
      Line(points = {{1, -80}, {10, -80}, {10, -48}, {18, -48}}, color = {255, 0, 255}));
    connect(pumHeaPumSou.m_flow_actual, tesEvaPum.u) annotation(
      Line(points = {{-35, -169}, {-35, -140}, {-30, -140}, {-30, -122}}, color = {0, 0, 127}));
    connect(and2.y, booToReaPum1.u) annotation(
      Line(points = {{41, -40}, {50, -40}, {50, -80}, {40, -80}, {40, -98}}, color = {255, 0, 255}));
    connect(preSou.ports[1], temRet.port_b) annotation(
      Line(points = {{70, -121}, {70, -122}, {60, -122}, {60, -42}}, color = {0, 127, 255}));
    connect(booToReaPum1.y, heaPum.TSet) annotation(
      Line(points = {{40, -121}, {40, -145}, {34, -145}}, color = {0, 0, 127}));
    connect(bou.ports[1], pumHeaPum1.port_a) annotation(
      Line(points = {{126, -6}, {90, -6}}, color = {0, 127, 255}));
    connect(booToReaPum.y, pumHeaPum1.m_flow_in) annotation(
      Line(points = {{-99, -110}, {-90, -110}, {-90, -226}, {108, -226}, {108, -34}, {80, -34}, {80, -18}}, color = {0, 0, 127}));
    connect(temRet1.port_b, preSou.ports[2]) annotation(
      Line(points = {{62, -148}, {64, -148}, {64, -119}, {70, -119}}, color = {0, 127, 255}));
    annotation(
      Documentation(info = "<html>
<p>
Example that simulates one room equipped with a radiator. Hot water is produced
by a <i>24</i> kW nominal capacity heat pump. The source side water temperature to the
heat pump is constant at <i>10</i>&deg;C.
</p>
<p>
The heat pump is turned on when the room temperature falls below
<i>19</i>&deg;C and turned
off when the room temperature rises above <i>21</i>&deg;C.
</p>
</html>", revisions = "<html>
<ul>
<li>
July 22, 2021, by Michael Wetter:<br/>
Removed assignments <code>pumHeaPum(y_start=1)</code> and <code>pumHeaPumSou(y_start=1)</code>.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/1498\">#1498</a>.
</li>
<li>
April 21, 2021, by Michael Wetter:<br/>
Corrected error in calculation of design mass flow rate.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2458\">#2458</a>.
</li>
<li>
May 2, 2019, by Jianjun Hu:<br/>
Replaced fluid source. This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1072\"> #1072</a>.
</li>
<li>
March 3, 2017, by Michael Wetter:<br/>
Changed mass flow test to use a hysteresis as a threshold test
can cause chattering.
</li>
<li>
January 27, 2017, by Massimo Cimmino:<br/>
First implementation.
</li>
</ul>
</html>"),
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-240, 120}, {160, -240}})),
      __Dymola_Commands(file = "modelica://Buildings/Resources/Scripts/Dymola/Fluid/HeatPumps/Examples/ScrollWaterToWater_OneRoomRadiator.mos" "Simulate and plot"),
      experiment(StopTime = 172800, Tolerance = 1e-08),
      uses(Buildings(version = "12.0.0")),
      version = "");
  end heatPumpAndStorage;

  partial model partialBaseCircuit
    Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 rad(redeclare package Medium = MediumW, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, Q_flow_nominal = Q_flow_nominal, T_a_nominal = TRadSup_nominal, T_b_nominal = TRadRet_nominal, m_flow_nominal = mHeaPum_flow_nominal, T_start = TRadSup_nominal) "Radiator" annotation(
      Placement(transformation(origin = {26, 24}, extent = {{0, -10}, {20, 10}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort temSup(redeclare package Medium = MediumW, m_flow_nominal = mHeaPum_flow_nominal, T_start = TRadSup_nominal) "Supply water temperature" annotation(
      Placement(transformation(origin = {-42, 24}, extent = {{-10, -10}, {10, 10}})));
    Buildings.Fluid.Storage.StratifiedEnhancedInternalHex tan(redeclare package Medium = MediumW, redeclare package MediumHex = MediumW, m_flow_nominal = mHeaPum_flow_nominal, VTan = 0.3, hTan = 1.2, dIns = 0.01, hHex_a = 0.995, hHex_b = 0.1, TTan_nominal = 293.15, THex_nominal = 323.15, mHex_flow_nominal = 0.278, Q_flow_nominal = 0.278*4200*20) annotation(
      Placement(transformation(origin = {-14, -6}, extent = {{-10, -10}, {10, 10}})));
    Buildings.Fluid.Movers.FlowControlled_m_flow pumHeaPum1(redeclare package Medium = MediumW, T_start = TRadSup_nominal, energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState, m_flow_nominal = mHeaPum_flow_nominal, m_flow_start = 0.85, nominalValuesDefineDefaultPressureCurve = true, use_riseTime = false, dpMax = 4e6) annotation(
      Placement(transformation(origin = {80, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Buildings.Fluid.Movers.FlowControlled_m_flow pumHeaPum(redeclare package Medium = MediumW, m_flow_nominal = mHeaPum_flow_nominal, m_flow_start = 0.85, T_start = TRadSup_nominal, nominalValuesDefineDefaultPressureCurve = true, use_riseTime = false, energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState, dpMax = 1e6) "Pump for radiator side" annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-70, -110})));
    Buildings.Fluid.Sensors.TemperatureTwoPort temRet2(redeclare package Medium = MediumW, m_flow_nominal = mHeaPum_flow_nominal, T_start = TRadSup_nominal) "Return water temperature" annotation(
      Placement(transformation(origin = {-54, -148}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.HeatPumps.Carnot_TCon heaPum(redeclare package Medium1 = MediumW, redeclare package Medium2 = MediumW, QCon_flow_nominal = 20000, dp1_nominal = 2000, dp2_nominal = 2000) annotation(
      Placement(transformation(origin = {22, -154}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Buildings.Fluid.Sensors.TemperatureTwoPort temRet1(redeclare package Medium = MediumW, m_flow_nominal = mHeaPum_flow_nominal, T_start = TRadSup_nominal) "Return water temperature" annotation(
      Placement(transformation(origin = {52, -148}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.Sensors.TemperatureTwoPort temRet(redeclare package Medium = MediumW, m_flow_nominal = mHeaPum_flow_nominal, T_start = TRadSup_nominal) "Return water temperature" annotation(
      Placement(transformation(origin = {60, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Buildings.Fluid.Movers.FlowControlled_m_flow pumHeaPumSou(redeclare package Medium = MediumW, m_flow_start = 0.85, m_flow_nominal = mHeaPum_flow_nominal, nominalValuesDefineDefaultPressureCurve = true, use_riseTime = false, energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState) "Pump for heat pump source side" annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-30, -180})));
    Buildings.Fluid.Sources.Boundary_pT sou(redeclare package Medium = MediumW, T = 281.15, nPorts = 1) "Fluid source on source side" annotation(
      Placement(transformation(extent = {{-80, -210}, {-60, -190}})));
    Buildings.Fluid.Sources.Boundary_pT sin(redeclare package Medium = MediumW, T = 283.15, nPorts = 1) "Fluid sink on source side" annotation(
      Placement(transformation(origin = {-2, 2}, extent = {{80, -210}, {60, -190}})));
    replaceable package MediumW = Buildings.Media.Water "Medium model for water";
    parameter Modelica.Units.SI.HeatFlowRate Q_flow_nominal = 20000 "Nominal heat flow rate of radiator";
    parameter Modelica.Units.SI.Temperature TRadSup_nominal = 273.15 + 50 "Radiator nominal supply water temperature";
    parameter Modelica.Units.SI.Temperature TRadRet_nominal = 273.15 + 45 "Radiator nominal return water temperature";
    parameter Modelica.Units.SI.MassFlowRate mHeaPum_flow_nominal = Q_flow_nominal/4200/5 "Heat pump nominal mass flow rate";
  equation
    connect(tan.port_a, temSup.port_a) annotation(
      Line(points = {{-14, 4}, {-70, 4}, {-70, 24}, {-52, 24}}, color = {0, 127, 255}));
    connect(temSup.port_b, rad.port_a) annotation(
      Line(points = {{-32, 24}, {26, 24}}, color = {0, 127, 255}));
    connect(pumHeaPum1.port_b, tan.port_b) annotation(
      Line(points = {{70, -6}, {2, -6}, {2, -16}, {-14, -16}}, color = {0, 127, 255}));
    connect(rad.port_b, pumHeaPum1.port_a) annotation(
      Line(points = {{46, 24}, {90, 24}, {90, -6}}, color = {0, 127, 255}));
    connect(pumHeaPum.port_b, tan.portHex_a) annotation(
      Line(points = {{-70, -100}, {-70, -9.8}, {-24, -9.8}}, color = {0, 127, 255}));
    connect(pumHeaPum.port_a, temRet2.port_a) annotation(
      Line(points = {{-70, -120}, {-70, -148}, {-64, -148}}, color = {0, 127, 255}));
    connect(temRet2.port_b, heaPum.port_b1) annotation(
      Line(points = {{-44, -148}, {12, -148}}, color = {0, 127, 255}));
    connect(heaPum.port_a1, temRet1.port_a) annotation(
      Line(points = {{32, -148}, {42, -148}}, color = {0, 127, 255}));
    connect(tan.portHex_b, temRet.port_a) annotation(
      Line(points = {{-24, -14}, {-30, -14}, {-30, -22}, {60, -22}}, color = {0, 127, 255}));
    connect(sou.ports[1], pumHeaPumSou.port_a) annotation(
      Line(points = {{-60, -200}, {-30, -200}, {-30, -190}}, color = {0, 127, 255}));
    connect(heaPum.port_a2, pumHeaPumSou.port_b) annotation(
      Line(points = {{12, -160}, {-30, -160}, {-30, -170}}, color = {0, 127, 255}));
    connect(heaPum.port_b2, sin.ports[1]) annotation(
      Line(points = {{32, -160}, {40, -160}, {40, -198}, {58, -198}}, color = {0, 127, 255}));
    annotation(
      Diagram(coordinateSystem(extent = {{-120, -240}, {200, 100}})),
      Icon(coordinateSystem(extent = {{-120, -240}, {200, 100}})));
  end partialBaseCircuit;

  model TestBaseCircuit
    extends partialBaseCircuit(pumHeaPum(inputType = Buildings.Fluid.Types.InputType.Constant), pumHeaPum1(inputType = Buildings.Fluid.Types.InputType.Constant), pumHeaPumSou(inputType = Buildings.Fluid.Types.InputType.Constant), heaPum(show_T = true, use_eta_Carnot_nominal = true));
    Buildings.Fluid.Sources.Boundary_pT preSou(redeclare package Medium = MediumW, T = TRadSup_nominal, nPorts = 2) "Source for pressure and to account for thermal expansion of water" annotation(
      Placement(transformation(extent = {{122, -116}, {102, -96}})));
    replaceable Buildings.Fluid.Sources.Boundary_pT bou(redeclare package Medium = MediumW, nPorts = 1) constrainedby Buildings.Fluid.Sources.BaseClasses.PartialSource annotation(
       Placement(transformation(origin = {130, -6}, extent = {{10, -10}, {-10, 10}})));
    Modelica.Blocks.Sources.Constant const(k = 273 + 30) annotation(
      Placement(transformation(extent = {{-12, -120}, {8, -100}})));
  equation
    connect(preSou.ports[1], temRet.port_b) annotation(
      Line(points = {{102, -107}, {96, -107}, {96, -106}, {60, -106}, {60, -42}}, color = {0, 127, 255}));
    connect(temRet1.port_b, preSou.ports[2]) annotation(
      Line(points = {{62, -148}, {94, -148}, {94, -105}, {102, -105}}, color = {0, 127, 255}));
    connect(bou.ports[1], pumHeaPum1.port_a) annotation(
      Line(points = {{120, -6}, {90, -6}}, color = {0, 127, 255}));
    connect(const.y, heaPum.TSet) annotation(
      Line(points = {{9, -110}, {34, -110}, {34, -145}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)),
      experiment(StopTime = 86400, __Dymola_Algorithm = "Dassl"));
  end TestBaseCircuit;

  model BaseCircuit_volumes
    extends partialBaseCircuit(pumHeaPum(inputType = Buildings.Fluid.Types.InputType.Constant), pumHeaPum1(inputType = Buildings.Fluid.Types.InputType.Constant), pumHeaPumSou(inputType = Buildings.Fluid.Types.InputType.Constant));
    Buildings.Fluid.MixingVolumes.MixingVolume vol(redeclare package Medium = MediumW, m_flow_nominal = 1, V(displayUnit = "l") = 0.005, nPorts = 2) annotation(
      Placement(transformation(extent = {{116, -6}, {136, 14}})));
    Buildings.Fluid.MixingVolumes.MixingVolume vol1(redeclare package Medium = Buildings.Media.Water "Water", m_flow_nominal = 1, V(displayUnit = "l") = 0.005, nPorts = 3) annotation(
      Placement(transformation(extent = {{92, -98}, {112, -78}})));
    Modelica.Blocks.Sources.Constant const(k = 273 + 35) annotation(
      Placement(transformation(extent = {{-4, -120}, {16, -100}})));
    Buildings.Fluid.Storage.ExpansionVessel exp(redeclare package Medium = MediumW) annotation(
      Placement(transformation(extent = {{156, -4}, {176, 16}})));
    Buildings.Fluid.Storage.ExpansionVessel exp1(redeclare package Medium = MediumW, V_start(displayUnit = "l") = 0.01) annotation(
      Placement(transformation(extent = {{130, -110}, {150, -90}})));
  equation
    connect(pumHeaPum1.port_a, vol.ports[1]) annotation(
      Line(points = {{90, -6}, {110, -6}, {110, -12}, {125, -12}, {125, -6}}, color = {0, 127, 255}));
    connect(temRet.port_b, vol1.ports[1]) annotation(
      Line(points = {{60, -42}, {62, -42}, {62, -98}, {100.667, -98}}, color = {0, 127, 255}));
    connect(vol1.ports[2], temRet1.port_b) annotation(
      Line(points = {{102, -98}, {104, -98}, {104, -152}, {62, -152}, {62, -148}}, color = {0, 127, 255}));
    connect(const.y, heaPum.TSet) annotation(
      Line(points = {{17, -110}, {34, -110}, {34, -145}}, color = {0, 0, 127}));
    connect(exp.port_a, vol.ports[2]) annotation(
      Line(points = {{166, -4}, {166, -12}, {127, -12}, {127, -6}}, color = {0, 127, 255}));
    connect(exp1.port_a, vol1.ports[3]) annotation(
      Line(points = {{140, -110}, {103.333, -110}, {103.333, -98}}, color = {0, 127, 255}));
  end BaseCircuit_volumes;

  model TestBuildingSystemes
    package Medium1 = BuildingSystems.Media.Water;
    package Medium2 = BuildingSystems.Media.Air;
    BuildingSystems.Fluid.HeatPumps.Carnot_TCon heatPump(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, QCon_flow_nominal = 5000, dp1_nominal = 1000, dp2_nominal = 1000) annotation(
      Placement(transformation(origin = {0, 4}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Blocks.Sources.Constant TSet(k = 273.15 + 35.0) annotation(
      Placement(transformation(origin = {-12, 130}, extent = {{44, -112}, {36, -104}})));
    BuildingSystems.Fluid.Movers.FlowControlled_dp pump1(redeclare package Medium = Medium1, m_flow_nominal = 0.1, nominalValuesDefineDefaultPressureCurve = true) annotation(
      Placement(transformation(origin = {-48, 58}, extent = {{-10, -10}, {10, 10}})));
  BuildingSystems.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 radiator(redeclare package Medium = Medium1, Q_flow_nominal = 6000, T_a_nominal(displayUnit = "K") = 308.15, T_b_nominal(displayUnit = "K") = 298.15, dp_nominal = 1000, m_flow_nominal = 0.5, TAir_nominal(displayUnit = "K"), TRad_nominal(displayUnit = "K")) annotation(
      Placement(transformation(origin = {0, 58}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Storage.ExpansionVessel exp(redeclare package Medium = Medium1, V_start = 0.05) annotation(
      Placement(transformation(origin = {66, 38}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sources.Boundary_pT bou1(nPorts = 1, redeclare package Medium = Medium2) annotation(
      Placement(transformation(origin = {-48, -22}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant dpSet(k = 150000.0) annotation(
      Placement(transformation(origin = {180, 64}, extent = {{-204, 18}, {-216, 30}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort radTepm(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {28, 58}, extent = {{-10, -10}, {10, 10}})));
  BuildingSystems.Fluid.Sources.MassFlowSource_T boundary(nPorts = 1, m_flow = 0.5, use_T_in = false, redeclare package Medium = Medium2)  annotation(
      Placement(transformation(origin = {38, -2}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  equation
    connect(TSet.y, heatPump.TSet) annotation(
      Line(points = {{24, 22}, {12, 22}, {12, 14}}, color = {0, 0, 127}));
    connect(heatPump.port_a2, bou1.ports[1]) annotation(
      Line(points = {{-10, -2}, {-23, -2}, {-23, -22}, {-38, -22}}, color = {0, 127, 255}));
    connect(pump1.port_b, radiator.port_a) annotation(
      Line(points = {{-38, 58}, {-10, 58}}, color = {0, 127, 255}));
    connect(heatPump.port_b1, pump1.port_a) annotation(
      Line(points = {{-10, 10}, {-76, 10}, {-76, 58}, {-58, 58}}, color = {0, 127, 255}));
    connect(exp.port_a, heatPump.port_a1) annotation(
      Line(points = {{66, 28}, {66, 10}, {10, 10}}, color = {0, 127, 255}));
    connect(dpSet.y, pump1.dp_in) annotation(
      Line(points = {{-36, 88}, {-48, 88}, {-48, 70}}, color = {0, 0, 127}));
    connect(radTepm.port_a, radiator.port_b) annotation(
      Line(points = {{18, 58}, {10, 58}}, color = {0, 127, 255}));
    connect(radTepm.port_b, heatPump.port_a1) annotation(
      Line(points = {{38, 58}, {48, 58}, {48, 10}, {10, 10}}, color = {0, 127, 255}));
    connect(boundary.ports[1], heatPump.port_b2) annotation(
      Line(points = {{28, -2}, {10, -2}}, color = {0, 127, 255}));
    annotation(
      Diagram(coordinateSystem(extent = {{-80, 100}, {80, -40}})));
  end TestBuildingSystemes;
  annotation(
    uses(Modelica(version = "4.0.0"), Buildings(version = "12.0.0"), BuildingSystems(version = "2.0.0-beta")));
end HeatSimulations;
