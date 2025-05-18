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
    extends Modelica.Icons.Example;
    package Medium1 = BuildingSystems.Media.Water;
    package Medium2 = BuildingSystems.Media.Air;
    parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal = -5 "Temperature difference evaporator inlet-outlet";
    parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal = 10 "Temperature difference condenser outlet-inlet";
    parameter Modelica.Units.SI.HeatFlowRate QCon_flow_nominal = 100E3 "Evaporator heat flow rate";
    parameter Modelica.Units.SI.MassFlowRate m1_flow_nominal = QCon_flow_nominal/dTCon_nominal/4200 "Nominal mass flow rate at condenser";
    BuildingSystems.Fluid.HeatPumps.Carnot_TCon heatPump(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, QCon_flow_nominal = 5000, dp1_nominal = 1000, dp2_nominal = 1000, dTEva_nominal = dTEva_nominal, dTCon_nominal = dTCon_nominal, allowFlowReversal1 = true, allowFlowReversal2 = true, show_T = true) annotation(
      Placement(transformation(origin = {0, 6}, extent = {{10, -10}, {-10, 10}})));
    Modelica.Blocks.Sources.Constant TSet(k = 273.15 + 50) annotation(
      Placement(transformation(origin = {-22, 162}, extent = {{44, -112}, {36, -104}})));
    BuildingSystems.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 radiator(redeclare package Medium = Medium1, Q_flow_nominal = 6000, T_a_nominal(displayUnit = "K") = 308.15, T_b_nominal(displayUnit = "K") = 298.15, dp_nominal = 1000, m_flow_nominal = 0.5, TAir_nominal(displayUnit = "K"), TRad_nominal(displayUnit = "K")) annotation(
      Placement(transformation(origin = {26, 124}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Storage.ExpansionVessel exp(redeclare package Medium = Medium1, V_start = 1) annotation(
      Placement(transformation(origin = {106, 18}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sources.Boundary_pT bou1(nPorts = 1, redeclare package Medium = Medium2, T = 283.15) annotation(
      Placement(transformation(origin = {60, -24}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sources.MassFlowSource_T boundary(nPorts = 1, m_flow = 0.5, use_T_in = false, redeclare package Medium = Medium2, T = 281.15) annotation(
      Placement(transformation(origin = {-46, -26}, extent = {{10, -10}, {-10, 10}})));
    BuildingSystems.Fluid.Movers.FlowControlled_dp pump11(redeclare package Medium = Medium1, m_flow_nominal = 0.1, nominalValuesDefineDefaultPressureCurve = true) annotation(
      Placement(transformation(origin = {-2, 124}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant dpSet1(k = 150000.0) annotation(
      Placement(transformation(origin = {194, 156}, extent = {{-204, 18}, {-216, 30}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort radTemp(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {56, 124}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort temp(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {-38, 124}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Thermal.HeatTransfer.Sources.FixedTemperature TBCCon1(T(displayUnit = "K") = 283.15) annotation(
      Placement(transformation(origin = {42, 144}, extent = {{-32, 28}, {-20, 40}})));
    Modelica.Thermal.HeatTransfer.Sources.FixedTemperature TBCRad1(T(displayUnit = "K") = 283.15) annotation(
      Placement(transformation(origin = {42, 144}, extent = {{-32, 48}, {-20, 60}})));
  equation
    connect(TSet.y, heatPump.TSet) annotation(
      Line(points = {{14, 54}, {14, 37.5}, {12, 37.5}, {12, 15}}, color = {0, 0, 127}));
    connect(radiator.port_a, pump11.port_b) annotation(
      Line(points = {{16, 124}, {8, 124}}, color = {0, 127, 255}));
    connect(dpSet1.y, pump11.dp_in) annotation(
      Line(points = {{-23, 180}, {-23, 136}, {-2, 136}}, color = {0, 0, 127}));
    connect(radTemp.port_a, radiator.port_b) annotation(
      Line(points = {{46, 124}, {36, 124}}, color = {0, 127, 255}));
    connect(radTemp.port_b, heatPump.port_a1) annotation(
      Line(points = {{66, 124}, {62, 124}, {62, 12}, {10, 12}}, color = {0, 127, 255}));
    connect(temp.port_b, pump11.port_a) annotation(
      Line(points = {{-28, 124}, {-12, 124}}, color = {0, 127, 255}));
    connect(temp.port_a, heatPump.port_b1) annotation(
      Line(points = {{-48, 124}, {-46, 124}, {-46, 12}, {-10, 12}}, color = {0, 127, 255}));
    connect(radiator.heatPortRad, TBCRad1.port) annotation(
      Line(points = {{28, 132}, {40, 132}, {40, 198}, {22, 198}}, color = {191, 0, 0}));
    connect(radiator.heatPortCon, TBCCon1.port) annotation(
      Line(points = {{24, 132}, {24, 178}, {22, 178}}, color = {191, 0, 0}));
    connect(exp.port_a, heatPump.port_a1) annotation(
      Line(points = {{106, 8}, {10, 8}, {10, 12}}, color = {0, 127, 255}));
    connect(boundary.ports[1], heatPump.port_a2) annotation(
      Line(points = {{-56, -26}, {-102, -26}, {-102, 0}, {-10, 0}}, color = {0, 127, 255}));
    connect(heatPump.port_b2, bou1.ports[1]) annotation(
      Line(points = {{10, 0}, {90, 0}, {90, -24}, {70, -24}}, color = {0, 127, 255}));
    annotation(
      Diagram(coordinateSystem(extent = {{-80, 220}, {120, -40}})));
  end TestBuildingSystemes;

  model TestBuildingSystemes2
    extends Modelica.Icons.Example;
    package Medium1 = BuildingSystems.Media.Water;
    package Medium2 = BuildingSystems.Media.Air;
    parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal = -5 "Temperature difference evaporator inlet-outlet";
    parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal = 10 "Temperature difference condenser outlet-inlet";
    parameter Modelica.Units.SI.HeatFlowRate QCon_flow_nominal = 100E3 "Evaporator heat flow rate";
    parameter Modelica.Units.SI.MassFlowRate m1_flow_nominal = QCon_flow_nominal/dTCon_nominal/4200 "Nominal mass flow rate at condenser";
    BuildingSystems.Fluid.HeatPumps.Carnot_TCon heatPump(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, QCon_flow_nominal = 5000, dp1_nominal = 1000, dp2_nominal = 1000, dTEva_nominal = dTEva_nominal, dTCon_nominal = dTCon_nominal, allowFlowReversal1 = false, allowFlowReversal2 = false, show_T = true) annotation(
      Placement(transformation(origin = {0, 6}, extent = {{10, -10}, {-10, 10}})));
    Modelica.Blocks.Sources.Constant TSet(k = 273.15 + 70.0) annotation(
      Placement(transformation(origin = {6, 130}, extent = {{44, -112}, {36, -104}})));
    BuildingSystems.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 radiator(redeclare package Medium = Medium1, Q_flow_nominal = 15000, T_a_nominal = 273.15 + 50, T_b_nominal = 273.15 + 40, dp_nominal = 100, TAir_nominal(displayUnit = "degC"), TRad_nominal(displayUnit = "K")) annotation(
      Placement(transformation(origin = {54, 124}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Storage.ExpansionVessel exp(redeclare package Medium = Medium1, V_start = 1, p_start = 2e5, T_start = 283.15) annotation(
      Placement(transformation(origin = {102, 54}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sources.Boundary_pT bou1(nPorts = 1, redeclare package Medium = Medium2, T = 285.15) annotation(
      Placement(transformation(origin = {0, -22}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sources.MassFlowSource_T boundary(nPorts = 1, m_flow = 0.5, use_T_in = false, redeclare package Medium = Medium2, T = 283.15) annotation(
      Placement(transformation(origin = {-38, -24}, extent = {{10, -10}, {-10, 10}})));
    BuildingSystems.Technologies.ThermalStorages.FluidStorage storage(redeclare package Medium = Medium1, height = 2, redeclare package Medium_HX_1 = Medium1, redeclare package Medium_HX_2 = Medium1, HX_2 = false, redeclare BuildingSystems.Technologies.ThermalStorages.BaseClasses.BuoyancyModels.Buoyancy1 HeatBuoyancy, HX_1 = true, UA_HX_1 = 5000, V = 2) annotation(
      Placement(transformation(origin = {4, 88}, extent = {{10, -10}, {-10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort radTemp(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {78, 124}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort temp(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {28, 44}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {-24, 110}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Storage.ExpansionVessel exp1(redeclare package Medium = Medium1, T_start = 283.15, V_start = 0.1, p_start = 1.5e5) annotation(
      Placement(transformation(origin = {106, 82}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort radTempIN(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {28, 124}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.MixingVolumes.MixingVolume room(redeclare package Medium = Medium2, m_flow_nominal = 300*2*1.2/3600, V = 1176) annotation(
      Placement(transformation(origin = {78, 183}, extent = {{-8, -7}, {8, 7}})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heatCapacitor(C = 4.254e6) annotation(
      Placement(transformation(origin = {60, 212}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor(G = 600) annotation(
      Placement(transformation(origin = {2, 184}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature annotation(
      Placement(transformation(origin = {-28, 184}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant tempOut(k = 273.15 + 10.0) annotation(
      Placement(transformation(origin = {-94, 310}, extent = {{44, -112}, {36, -104}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor tempRoom annotation(
      Placement(transformation(origin = {104, 198}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Logical.Hysteresis hysRad(uHigh = 273.15 + 25, uLow = 273.15 + 21, pre_y_start = false) annotation(
      Placement(transformation(origin = {230, 310}, extent = {{-80, -120}, {-60, -100}})));
    Modelica.Blocks.Logical.Not not1 annotation(
      Placement(transformation(origin = {232, 310}, extent = {{-40, -120}, {-20, -100}})));
    Modelica.Blocks.Math.BooleanToInteger booToInt annotation(
      Placement(transformation(origin = {230, 318}, extent = {{0, -160}, {20, -140}})));
    BuildingSystems.Fluid.Movers.FlowControlled_m_flow pumpRad(energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState, m_flow_nominal = 0.357, inputType = BuildingSystems.Fluid.Types.InputType.Stages, redeclare package Medium = Medium1) annotation(
      Placement(transformation(origin = {0, 124}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Logical.Hysteresis hysRad1(uHigh = 273.15 + 60, uLow = 273.15 + 50, pre_y_start = true) annotation(
      Placement(transformation(origin = {-102, 258}, extent = {{-80, -120}, {-60, -100}})));
    Modelica.Blocks.Logical.Not not11 annotation(
      Placement(transformation(origin = {-100, 258}, extent = {{-40, -120}, {-20, -100}})));
    Modelica.Blocks.Math.BooleanToInteger booToInt1 annotation(
      Placement(transformation(origin = {-102, 266}, extent = {{0, -160}, {20, -140}})));
    BuildingSystems.Fluid.Movers.FlowControlled_m_flow pumpRad1(energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState, inputType = BuildingSystems.Fluid.Types.InputType.Stages, m_flow_nominal = 0.2, redeclare package Medium = Medium1) annotation(
      Placement(transformation(origin = {-68, 84}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(TSet.y, heatPump.TSet) annotation(
      Line(points = {{42, 22}, {42, 15}, {12, 15}}, color = {0, 0, 127}));
    connect(exp.port_a, heatPump.port_a1) annotation(
      Line(points = {{102, 44}, {102, 12}, {10, 12}}, color = {0, 127, 255}));
    connect(temp.port_b, exp.port_a) annotation(
      Line(points = {{38, 44}, {102, 44}}, color = {0, 127, 255}));
    connect(radTemp.port_a, radiator.port_b) annotation(
      Line(points = {{68, 124}, {64, 124}}, color = {0, 127, 255}));
    connect(heatPump.port_a2, boundary.ports[1]) annotation(
      Line(points = {{-10, 0}, {-56, 0}, {-56, -24}, {-48, -24}}, color = {0, 127, 255}));
    connect(heatPump.port_b2, bou1.ports[1]) annotation(
      Line(points = {{10, 0}, {20, 0}, {20, -22}, {10, -22}}, color = {0, 127, 255}));
    connect(storage.port_HX_1_b, temp.port_a) annotation(
      Line(points = {{-3, 82}, {-26, 82}, {-26, 44}, {18, 44}}, color = {0, 127, 255}));
    connect(radiator.port_a, radTempIN.port_b) annotation(
      Line(points = {{44, 124}, {38, 124}}, color = {0, 127, 255}));
    connect(radiator.heatPortCon, room.heatPort) annotation(
      Line(points = {{52, 132}, {52, 183}, {70, 183}}, color = {191, 0, 0}));
    connect(radiator.heatPortRad, room.heatPort) annotation(
      Line(points = {{56, 132}, {60, 132}, {60, 183}, {70, 183}}, color = {191, 0, 0}));
    connect(heatCapacitor.port, room.heatPort) annotation(
      Line(points = {{60, 202}, {70, 202}, {70, 184}}, color = {191, 0, 0}));
    connect(room.heatPort, thermalConductor.port_b) annotation(
      Line(points = {{70, 184}, {12, 184}}, color = {191, 0, 0}));
    connect(thermalConductor.port_a, prescribedTemperature.port) annotation(
      Line(points = {{-8, 184}, {-18, 184}}, color = {191, 0, 0}));
    connect(tempOut.y, prescribedTemperature.T) annotation(
      Line(points = {{-58, 202}, {-64, 202}, {-64, 184}, {-40, 184}}, color = {0, 0, 127}));
    connect(tempRoom.port, room.heatPort) annotation(
      Line(points = {{94, 198}, {70, 198}, {70, 184}}, color = {191, 0, 0}));
    connect(tempRoom.T, hysRad.u) annotation(
      Line(points = {{116, 198}, {148, 198}, {148, 200}}, color = {0, 0, 127}));
    connect(hysRad.y, not1.u) annotation(
      Line(points = {{172, 200}, {190, 200}}, color = {255, 0, 255}));
    connect(not1.y, booToInt.u) annotation(
      Line(points = {{214, 200}, {222, 200}, {222, 168}, {228, 168}}, color = {255, 0, 255}));
    connect(senTem.port_a, pumpRad.port_a) annotation(
      Line(points = {{-34, 110}, {-40, 110}, {-40, 124}, {-10, 124}}, color = {0, 127, 255}));
    connect(pumpRad.port_b, radTempIN.port_a) annotation(
      Line(points = {{10, 124}, {18, 124}}, color = {0, 127, 255}));
    connect(booToInt.y, pumpRad.stage) annotation(
      Line(points = {{251, 168}, {260, 168}, {260, 150}, {0, 150}, {0, 136}}, color = {255, 127, 0}));
    connect(hysRad1.y, not11.u) annotation(
      Line(points = {{-161, 148}, {-143, 148}}, color = {255, 0, 255}));
    connect(not11.y, booToInt1.u) annotation(
      Line(points = {{-119, 148}, {-111, 148}, {-111, 116}, {-105, 116}}, color = {255, 0, 255}));
    connect(storage.port_HX_1_a, pumpRad1.port_b) annotation(
      Line(points = {{-3, 84}, {-58, 84}}, color = {0, 127, 255}));
    connect(pumpRad1.port_a, heatPump.port_b1) annotation(
      Line(points = {{-78, 84}, {-86, 84}, {-86, 12}, {-10, 12}}, color = {0, 127, 255}));
    connect(storage.port_b1, senTem.port_b) annotation(
      Line(points = {{11, 97}, {-14, 97}, {-14, 110}}, color = {0, 127, 255}));
    connect(storage.port_a1, radTemp.port_b) annotation(
      Line(points = {{11, 79}, {88, 79}, {88, 124}}, color = {0, 127, 255}));
    connect(exp1.port_a, storage.port_a1) annotation(
      Line(points = {{106, 72}, {11, 72}, {11, 79}}, color = {0, 127, 255}));
    connect(booToInt1.y, pumpRad1.stage) annotation(
      Line(points = {{-80, 116}, {-68, 116}, {-68, 96}}, color = {255, 127, 0}));
    connect(storage.T[1], hysRad1.u) annotation(
      Line(points = {{12, 94}, {18, 94}, {18, 58}, {-194, 58}, {-194, 148}, {-184, 148}}, color = {0, 0, 127}));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, 260}, {260, -40}})));
  end TestBuildingSystemes2;

  model CommunitySystem "Photovoltaic cooling system"
    extends Modelica.Icons.Example;
    package Medium1 = BuildingSystems.Media.Water;
    package Medium2 = BuildingSystems.Media.Air;
    parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal = -5 "Temperature difference evaporator inlet-outlet";
    parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal = 10 "Temperature difference condenser outlet-inlet";
    parameter Modelica.Units.SI.HeatFlowRate QCon_flow_nominal = 100E3 "Evaporator heat flow rate";
    parameter Modelica.Units.SI.MassFlowRate m1_flow_nominal = QCon_flow_nominal/dTCon_nominal/4200 "Nominal mass flow rate at condenser";
    parameter Modelica.Units.SI.MassFlowRate m_flow_nominal = 0.5 "Nominal mass flow rate";
    parameter Modelica.Units.SI.MassFlowRate m_flow = 0.5 "Mass flow rate in the cold water production and consumption loop";
    BuildingSystems.Technologies.Photovoltaics.PVModules.PVModuleSimpleMPP pvField(redeclare BuildingSystems.Technologies.Photovoltaics.Data.PhotovoltaicModules.TSM230PC05 pvModuleData, angleDegTil_constant = 30.0, angleDegAzi_constant = 0.0, nModPar = 8, nModSer = 1) annotation(
      Placement(transformation(extent = {{-54, -18}, {-34, 2}})));
    BuildingSystems.Climate.SolarRadiationTransformers.SolarRadiationTransformerIsotropicSky radiation(rhoAmb = 0.2, angleDegL = 0.0) annotation(
      Placement(transformation(extent = {{-72, 12}, {-52, 32}})));
    BuildingSystems.Technologies.ElectricalStorages.BatterySimple battery(redeclare BuildingSystems.Technologies.ElectricalStorages.Data.LeadAcid.LeadAcidGeneric batteryData, nBat = 3) annotation(
      Placement(transformation(origin = {4, -74}, extent = {{-38, -10}, {-18, 10}})));
    BuildingSystems.Buildings.BuildingTemplates.Building1Zone1DDistrict building(calcIdealLoads = false, heatSources = true, nHeatSources = 1, angleDegAziBuilding = 0.0, width = 10, length = 14, heightSto = 2.8, nSto = 3, ARoom = 70, widthWindow1 = 4.0, heightWindow1 = 2*1.2, widthWindow2 = 4.0, heightWindow2 = 2*1.2, widthWindow3 = 4.0, heightWindow3 = 2*1.2, widthWindow4 = 4.0, heightWindow4 = 2*1.2, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.OuterWallMultistorey1958to1968 constructionWall1, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.OuterWallMultistorey1958to1968 constructionWall2, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.OuterWallMultistorey1958to1968 constructionWall3, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.OuterWallMultistorey1958to1968 constructionWall4, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.BasePlateMultistorey1958to1968 constructionBottom, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.IntermediateWallMultistorey1958to1968 constructionWallsInterior, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.IntermediateCeilingMultistorey1958to1968 constructionCeilingsInterior, redeclare BuildingSystems.Buildings.Data.Constructions.Transparent.DoubleGlazing constructionWindow2, redeclare BuildingSystems.Buildings.Data.Constructions.Transparent.DoubleGlazing constructionWindow3, redeclare BuildingSystems.Buildings.Data.Constructions.Transparent.DoubleGlazing constructionWindow4, redeclare BuildingSystems.Buildings.Data.Constructions.Transparent.DoubleGlazing constructionWindow1, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.RoofRowhouse1918 constructionCeiling, BCWall1 = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, BCWall2 = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, BCWall3 = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, BCWall4 = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, BCCeiling = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience) "Building model" annotation(
      Placement(transformation(extent = {{88, 30}, {108, 50}})));
    BuildingSystems.Buildings.Ambience ambience(nSurfaces = building.nSurfacesAmbience, redeclare block WeatherData = BuildingSystems.Climate.WeatherDataMeteonorm.Germany_Berlin_Meteonorm_ASCII) "Ambience model" annotation(
      Placement(transformation(extent = {{64, 30}, {84, 50}})));
    Modelica.Blocks.Sources.Constant airchange(k = 0.5) annotation(
      Placement(transformation(origin = {116, 44}, extent = {{-2, -2}, {2, 2}}, rotation = 180)));
    Modelica.Units.SI.Energy EGrid(start = 0.0) "Integrates the electricity taken from the grid";
    Modelica.Units.SI.Energy EPVField(start = 0.0) "Integrates the electricity generated by the PV field";
    BuildingSystems.Fluid.Sources.Boundary_pT bou1(redeclare package Medium = Medium2, nPorts = 1) annotation(
      Placement(transformation(origin = {108, -80}, extent = {{6, -6}, {-6, 6}})));
    BuildingSystems.Fluid.Sources.MassFlowSource_T boundary(redeclare package Medium = Medium2, m_flow = 0.5, nPorts = 1, use_T_in = true) annotation(
      Placement(transformation(origin = {20, -92}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.HeatPumps.Carnot_TCon heatPump(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, QCon_flow_nominal = 5000, allowFlowReversal1 = false, allowFlowReversal2 = false, dTCon_nominal = dTCon_nominal, dTEva_nominal = dTEva_nominal, dp1_nominal = 1000, dp2_nominal = 1000, show_T = true) annotation(
      Placement(transformation(origin = {70, -74}, extent = {{10, -10}, {-10, 10}})));
    BuildingSystems.Fluid.Movers.FlowControlled_dp pump1(redeclare package Medium = Medium1, m_flow_nominal = 0.1, nominalValuesDefineDefaultPressureCurve = true, p_start = 1.8e5) annotation(
      Placement(transformation(origin = {36, -26}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant dpSet(k = 150000.0) annotation(
      Placement(transformation(origin = {254, -18}, extent = {{-204, 18}, {-216, 30}})));
    BuildingSystems.Technologies.ThermalStorages.FluidStorage storage(HX_2 = false, redeclare BuildingSystems.Technologies.ThermalStorages.BaseClasses.BuoyancyModels.Buoyancy1 HeatBuoyancy, redeclare package Medium = Medium1, redeclare package Medium_HX_1 = Medium1, redeclare package Medium_HX_2 = Medium2, height = 2) annotation(
      Placement(transformation(origin = {72, -22}, extent = {{10, -10}, {-10, 10}})));
    Modelica.Blocks.Sources.Constant TSet(k = 273.15 + 35.0) annotation(
      Placement(transformation(origin = {62, 56}, extent = {{44, -112}, {36, -104}})));
    BuildingSystems.Fluid.Storage.ExpansionVessel exp(redeclare package Medium = Medium1, V_start = 1, p_start = 1.8e5) annotation(
      Placement(transformation(origin = {164, -58}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort temp(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {122, -24}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 radiator(redeclare package Medium = Medium1, Q_flow_nominal = 6000, TAir_nominal(displayUnit = "K"), TRad_nominal(displayUnit = "K"), T_a_nominal(displayUnit = "K") = 308.15, T_b_nominal(displayUnit = "K") = 298.15, dp_nominal = 1000, m_flow_nominal = 0.5, p_start = 1.5e5) annotation(
      Placement(transformation(origin = {188, 20}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Storage.ExpansionVessel exp1(redeclare package Medium = Medium1, V_start = 0.05, p_start = 1.5e5) annotation(
      Placement(transformation(origin = {256, 30}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Movers.FlowControlled_dp pump11(redeclare package Medium = Medium1, m_flow_nominal = 0.1, nominalValuesDefineDefaultPressureCurve = true, p_start = 1.5e5) annotation(
      Placement(transformation(origin = {152, 20}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant dpSet1(k = 150000.0) annotation(
      Placement(transformation(origin = {380, 26}, extent = {{-204, 18}, {-216, 30}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort radTemp(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {220, 20}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.CombiTimeTable hotWaterConsumption(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, smoothness = Modelica.Blocks.Types.Smoothness.ConstantSegments, table = [7*3600, 420]) "Spotřeba 12 lidí" annotation(
      Placement(transformation(origin = {214, -156}, extent = {{-20, 70}, {0, 90}})));
    BuildingSystems.Fluid.Sources.Boundary_pT bou2(redeclare package Medium = Medium1, nPorts = 1, p = 1.5e5, use_X_in = false, use_Xi_in = true) annotation(
      Placement(transformation(origin = {222, -42}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));
    Buildings.Fluid.Sources.MassFlowSource_T boundary2(redeclare package Medium = Medium1, T = 283.15, m_flow = 0.5, nPorts = 1) annotation(
      Placement(transformation(origin = {252, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    BuildingSystems.Electrical.AC.ThreePhasesBalanced.Loads.Inductive load(use_pf_in = true) annotation(
      Placement(transformation(origin = {-28, -110}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Electrical.AC.ThreePhasesBalanced.Sources.FixedVoltage fixVol(V = 480, f = 60, phiSou = 0.3490658503988659) annotation(
      Placement(transformation(origin = {-68, -110}, extent = {{-10, -10}, {10, 10}})));
  equation
    der(EGrid) = battery.PGrid;
    der(EPVField) = pvField.PField;
    connect(radiation.radiationPort, pvField.radiationPort) annotation(
      Line(points = {{-54, 21.8}, {-46, 21.8}, {-46, 0}}, color = {255, 255, 0}, smooth = Smooth.None));
    connect(pvField.PField, battery.PCharge) annotation(
      Line(points = {{-38, 0}, {-35.5, 0}, {-35.5, -74}, {-29, -74}}, color = {0, 0, 127}));
    connect(ambience.toSurfacePorts, building.toAmbienceSurfacesPorts) annotation(
      Line(points = {{82, 44}, {89, 44}}, color = {0, 255, 0}));
    connect(ambience.toAirPorts, building.toAmbienceAirPorts) annotation(
      Line(points = {{82, 36}, {89, 36}}, color = {85, 170, 255}));
    connect(building.airchange[1], airchange.y) annotation(
      Line(points = {{107.8, 44}, {113.8, 44}}, color = {0, 0, 127}));
    connect(ambience.TAirRef, building.TAirAmb) annotation(
      Line(points = {{65.8, 47}, {62, 47}, {62, 56}, {104.2, 56}, {104.2, 49.8}}, color = {0, 0, 127}));
    connect(ambience.xAir, building.xAirAmb) annotation(
      Line(points = {{65.8, 45}, {60, 45}, {60, 60}, {106.4, 60}, {106.4, 49.8}}, color = {0, 0, 127}));
    connect(ambience.IrrDirHor, radiation.IrrDirHor) annotation(
      Line(points = {{65.8, 43}, {-74, 43}, {-74, 28}, {-69.6, 28}}, color = {0, 0, 127}));
    connect(radiation.IrrDifHor, ambience.IrrDifHor) annotation(
      Line(points = {{-69.6, 24}, {-76, 24}, {-76, 42}, {65.8, 42}, {65.8, 41}}, color = {0, 0, 127}));
    connect(ambience.TAirRef, pvField.TAmb) annotation(
      Line(points = {{65.8, 47}, {-42, 47}, {-42, 0}}, color = {0, 0, 127}));
    connect(pvField.angleDegTil, radiation.angleDegTil) annotation(
      Line(points = {{-50, 0}, {-76, 0}, {-76, 20}, {-69.6, 20}}, color = {0, 0, 127}));
    connect(pvField.angleDegAzi, radiation.angleDegAzi) annotation(
      Line(points = {{-50, -2}, {-50, -2}, {-76, -2}, {-76, 16}, {-69.6, 16}}, color = {0, 0, 127}));
    connect(ambience.latitudeDeg, radiation.latitudeDeg) annotation(
      Line(points = {{67, 49}, {67, 64}, {-65.8, 64}, {-65.8, 29.6}}, color = {0, 0, 127}));
    connect(ambience.longitudeDeg, radiation.longitudeDeg) annotation(
      Line(points = {{69, 49}, {69, 64}, {-62, 64}, {-62, 29.6}}, color = {0, 0, 127}));
    connect(ambience.longitudeDeg0, radiation.longitudeDeg0) annotation(
      Line(points = {{71, 49}, {71, 64}, {-58, 64}, {-58, 29.6}}, color = {0, 0, 127}));
    connect(heatPump.port_b2, bou1.ports[1]) annotation(
      Line(points = {{80, -80}, {102, -80}}, color = {0, 127, 255}));
    connect(heatPump.port_a2, boundary.ports[1]) annotation(
      Line(points = {{60, -80}, {45, -80}, {45, -92}, {30, -92}}, color = {0, 127, 255}));
    connect(dpSet.y, pump1.dp_in) annotation(
      Line(points = {{37, 6}, {37, -14}, {36.4, -14}}, color = {0, 0, 127}));
    connect(heatPump.port_b1, pump1.port_a) annotation(
      Line(points = {{60, -68}, {8, -68}, {8, -26}, {26, -26}}, color = {0, 127, 255}));
    connect(pump1.port_b, storage.port_HX_1_a) annotation(
      Line(points = {{46, -26}, {65, -26}}, color = {0, 127, 255}));
    connect(storage.port_HX_1_b, temp.port_a) annotation(
      Line(points = {{65, -28}, {54, -28}, {54, -42}, {88, -42}, {88, -24}, {112, -24}}, color = {0, 127, 255}));
    connect(temp.port_b, heatPump.port_a1) annotation(
      Line(points = {{132, -24}, {148, -24}, {148, -68}, {80, -68}}, color = {0, 127, 255}));
    connect(TSet.y, heatPump.TSet) annotation(
      Line(points = {{98, -52}, {82, -52}, {82, -64}}, color = {0, 0, 127}));
    connect(exp.port_a, heatPump.port_a1) annotation(
      Line(points = {{164, -68}, {80, -68}}, color = {0, 127, 255}));
    connect(heatPump.P, battery.PLoad) annotation(
      Line(points = {{60, -74}, {-19, -74}}, color = {0, 0, 127}));
    connect(radiator.port_a, pump11.port_b) annotation(
      Line(points = {{178, 20}, {162, 20}}, color = {0, 127, 255}));
    connect(dpSet1.y, pump11.dp_in) annotation(
      Line(points = {{163.4, 50}, {151.4, 50}, {151.4, 32}}, color = {0, 0, 127}));
    connect(radTemp.port_a, radiator.port_b) annotation(
      Line(points = {{210, 20}, {198, 20}}, color = {0, 127, 255}));
    connect(radTemp.port_b, exp1.port_a) annotation(
      Line(points = {{230, 20}, {256, 20}}, color = {0, 127, 255}));
    connect(storage.port_a1, pump11.port_a) annotation(
      Line(points = {{79, -31}, {86, -31}, {86, 20}, {142, 20}}, color = {0, 127, 255}));
    connect(exp1.port_a, storage.port_a2) annotation(
      Line(points = {{256, 20}, {256, -6}, {65, -6}, {65, -13}}, color = {0, 127, 255}));
    connect(radiator.heatPortRad, building.conHeatSourcesPorts[1]) annotation(
      Line(points = {{190, 28}, {190, 78}, {98, 78}, {98, 50}}, color = {191, 0, 0}));
    connect(radiator.heatPortCon, building.radHeatSourcesPorts[1]) annotation(
      Line(points = {{186, 28}, {186, 76}, {100, 76}, {100, 50}}, color = {191, 0, 0}));
    connect(boundary.T_in, ambience.TAirRef) annotation(
      Line(points = {{8, -88}, {0, -88}, {0, 48}, {66, 48}}, color = {0, 0, 127}));
    connect(pump11.port_b, bou2.ports[1]) annotation(
      Line(points = {{162, 20}, {162, -34}, {222, -34}}, color = {0, 127, 255}));
    connect(bou2.X_in, hotWaterConsumption.y) annotation(
      Line(points = {{225, -52}, {224.5, -52}, {224.5, -76}, {215, -76}}, color = {0, 0, 127}, thickness = 0.5));
    connect(boundary2.ports[1], storage.port_a2) annotation(
      Line(points = {{252, -18}, {252, -6}, {66, -6}, {66, -12}}, color = {0, 127, 255}));
    connect(fixVol.terminal, load.terminal) annotation(
      Line(points = {{-58, -110}, {-38, -110}}, color = {0, 170, 0}));
    connect(load.pf_in, battery.PGrid) annotation(
      Line(points = {{-18, -104}, {-10, -104}, {-10, -70}, {-18, -70}}, color = {0, 0, 127}));
    annotation(
      experiment(StartTime = 0, StopTime = 31536000),
      __Dymola_Commands(file = "modelica://BuildingSystems/Resources/Scripts/Dymola/Applications/AirConditioningSystems/PhotovoltaicCoolingSystem.mos" "Simulate and plot"),
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-80, 80}, {280, -100}})),
      Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}})),
      Documentation(info = "<html>
  <p>
  Example that simulates a photovoltaic driven air-conditioning system:
  The PV generator loads an electric battery. The compression chiller takes
  the electricity from the battery, produces cold water and charges it
  into a cold water storage. The cold water is used for the air-conditioning of a building.
  </p>
  </html>", revisions = "<html>
  <ul>
  <li>
  August 18, 2018, by Christoph Nytsch-Geusen:<br/>
  Adapted to possible different media for the storage content and the two internal heat exchangers.
  </li>
  <li>
  May 21, 2016, by Christoph Nytsch-Geusen:<br/>
  First implementation.
  </li>
  </ul>
  </html>"),
      uses(BuildingSystems(version = "2.0.0-beta")),
      version = "");
  end CommunitySystem;

  model CommunitySystemOnGrid "Photovoltaic cooling system"
    extends Modelica.Icons.Example;
    package Medium1 = BuildingSystems.Media.Water;
    package Medium2 = BuildingSystems.Media.Air;
    parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal = -5 "Temperature difference evaporator inlet-outlet";
    parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal = 10 "Temperature difference condenser outlet-inlet";
    parameter Modelica.Units.SI.HeatFlowRate QCon_flow_nominal = 20E3 "Evaporator heat flow rate, změněno ze 100";
    parameter Modelica.Units.SI.MassFlowRate m1_flow_nominal = QCon_flow_nominal/dTCon_nominal/4200 "Nominal mass flow rate at condenser";
    parameter Modelica.Units.SI.MassFlowRate m_flow_nominal = 0.5 "Nominal mass flow rate";
    BuildingSystems.Technologies.Photovoltaics.PVModules.PVModuleSimpleMPP pvField(redeclare BuildingSystems.Technologies.Photovoltaics.Data.PhotovoltaicModules.ASE300DGFT315Wp pvModuleData, angleDegTil_constant = 30.0, angleDegAzi_constant = 0.0, nModPar = 11, nModSer = 5) "Tolik panelů, že to zpalní střechu 140m^2" annotation(
      Placement(transformation(origin = {18, -14}, extent = {{-54, -18}, {-34, 2}})));
    BuildingSystems.Climate.SolarRadiationTransformers.SolarRadiationTransformerIsotropicSky radiation(rhoAmb = 0.2, angleDegL = 0.0) annotation(
      Placement(transformation(origin = {18, -14}, extent = {{-72, 12}, {-52, 32}})));
    BuildingSystems.Technologies.ElectricalStorages.BatterySimple battery(redeclare BuildingSystems.Technologies.ElectricalStorages.Data.LithiumIon.LithiumIonTeslaPowerwall2 batteryData, nBat = 1) annotation(
      Placement(transformation(origin = {10, -72}, extent = {{-38, -10}, {-18, 10}})));
    BuildingSystems.Buildings.BuildingTemplates.Building1Zone1DDistrict building(calcIdealLoads = false, heatSources = true, nHeatSources = 1, angleDegAziBuilding = 0.0, width = 10, length = 14, heightSto = 2.8, nSto = 3, ARoom = 70, widthWindow1 = 4, heightWindow1 = 2*1.2, widthWindow2 = 4, heightWindow2 = 2*1.2, widthWindow3 = 4, heightWindow3 = 2*1.2, widthWindow4 = 4, heightWindow4 = 2*1.2, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.IntermediateWallSingle2014 constructionWall1, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.IntermediateWallSingle2014 constructionWall2, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.IntermediateWallSingle2014 constructionWall3, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.IntermediateWallSingle2014 constructionWall4, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.BasePlateSingle2014 constructionBottom, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.IntermediateWallMultistorey1958to1968 constructionWallsInterior, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.IntermediateCeilingMultistorey1958to1968 constructionCeilingsInterior, redeclare BuildingSystems.Buildings.Data.Constructions.Transparent.DoubleGlazing constructionWindow2, redeclare BuildingSystems.Buildings.Data.Constructions.Transparent.DoubleGlazing constructionWindow3, redeclare BuildingSystems.Buildings.Data.Constructions.Transparent.DoubleGlazing constructionWindow4, redeclare BuildingSystems.Buildings.Data.Constructions.Transparent.DoubleGlazing constructionWindow1, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.IntermediateCeilingSingle2014 constructionCeiling, BCWall1 = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, BCWall2 = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, BCWall3 = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, BCWall4 = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, BCCeiling = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, nZones = 1) "6 bytů. každý 70m^2" annotation(
      Placement(transformation(origin = {18, -14}, extent = {{88, 30}, {108, 50}})));
    BuildingSystems.Buildings.Ambience ambience(nSurfaces = building.nSurfacesAmbience, redeclare block WeatherData = BuildingSystems.Climate.WeatherDataMeteonorm.Germany_Berlin_Meteonorm_ASCII) "Ambience model" annotation(
      Placement(transformation(origin = {18, -14}, extent = {{64, 30}, {84, 50}})));
    Modelica.Blocks.Sources.Constant airchange(k = 0.5) "větrání, kolikrát za hodinu se vymění objem vzducuhu v budově" annotation(
      Placement(transformation(origin = {134, 30}, extent = {{-2, -2}, {2, 2}}, rotation = 180)));
    Modelica.Units.SI.Energy EGrid(start = 0.0) "Integrates the electricity taken from the grid";
    Modelica.Units.SI.Energy EPVField(start = 0.0) "Integrates the electricity generated by the PV field";
    BuildingSystems.Fluid.HeatPumps.Carnot_TCon heatPump(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, QCon_flow_nominal = 3000, allowFlowReversal1 = false, allowFlowReversal2 = false, dp1_nominal = 1000, dp2_nominal = 1000, show_T = true) annotation(
      Placement(transformation(origin = {132, -132}, extent = {{10, -10}, {-10, 10}})));
    Modelica.Blocks.Sources.Constant TSet(k = 273.15 + 70.0) annotation(
      Placement(transformation(origin = {140, -6}, extent = {{44, -112}, {36, -104}})));
    BuildingSystems.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 radiator(redeclare package Medium = Medium1, Q_flow_nominal = 40000, TAir_nominal(displayUnit = "degC") = 294.15, TRad_nominal(displayUnit = "K"), T_a_nominal = 273.15 + 50, T_b_nominal = 273.15 + 40, dp_nominal = 100) annotation(
      Placement(transformation(origin = {188, -26}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Storage.ExpansionVessel expHP(redeclare package Medium = Medium1, T_start = 283.15, V_start = 1, p_start = 2e5) annotation(
      Placement(transformation(origin = {202, -92}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sources.Boundary_pT boundary2(redeclare package Medium = Medium2, T = 285.15, nPorts = 1) annotation(
      Placement(transformation(origin = {182, -148}, extent = {{10, -10}, {-10, 10}})));
    BuildingSystems.Fluid.Sources.MassFlowSource_T boundary(redeclare package Medium = Medium2, T = 283.15, m_flow = 0.5, nPorts = 1, use_T_in = true) annotation(
      Placement(transformation(origin = {88, -150}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Technologies.ThermalStorages.FluidStorage storage(HX_1 = true, HX_2 = false, redeclare BuildingSystems.Technologies.ThermalStorages.BaseClasses.BuoyancyModels.Buoyancy1 HeatBuoyancy, redeclare package Medium = Medium1, redeclare package Medium_HX_1 = Medium1, redeclare package Medium_HX_2 = Medium1, UA_HX_1 = 3000, V = 8, height = 2) annotation(
      Placement(transformation(origin = {136, -62}, extent = {{10, -10}, {-10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort tempRadOUT(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {212, -26}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort tempHP(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {162, -102}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort tempStorageOUT(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Storage.ExpansionVessel expRad(redeclare package Medium = Medium1, T_start = 283.15, V_start = 0.1, p_start = 1.5e5) annotation(
      Placement(transformation(origin = {240, -68}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort tempRadIN(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {162, -26}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Logical.Hysteresis hysRad(pre_y_start = false, uHigh = 273.15 + 25, uLow = 273.15 + 21) annotation(
      Placement(transformation(origin = {122, 64}, extent = {{-48, -72}, {-36, -60}})));
    Modelica.Blocks.Logical.Not not1 annotation(
      Placement(transformation(origin = {122, 64}, extent = {{-24, -72}, {-12, -60}})));
    Modelica.Blocks.Math.BooleanToInteger booToInt annotation(
      Placement(transformation(origin = {118, 88}, extent = {{0, -96}, {12, -84}})));
    BuildingSystems.Fluid.Movers.FlowControlled_m_flow pumpRad(redeclare package Medium = Medium1, energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState, inputType = BuildingSystems.Fluid.Types.InputType.Stages, m_flow_nominal = 0.5) annotation(
      Placement(transformation(origin = {134, -26}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Logical.Hysteresis hysRad1(pre_y_start = true, uHigh = 273.15 + 60, uLow = 273.15 + 50) annotation(
      Placement(transformation(origin = {70, 34}, extent = {{-48, -72}, {-36, -60}})));
    Modelica.Blocks.Logical.Not not11 annotation(
      Placement(transformation(origin = {68, 34}, extent = {{-24, -72}, {-12, -60}})));
    Modelica.Blocks.Math.BooleanToInteger booToInt1 annotation(
      Placement(transformation(origin = {64, 42}, extent = {{0, -80}, {10, -70}})));
    BuildingSystems.Fluid.Movers.FlowControlled_m_flow pumpHP(redeclare package Medium = Medium1, energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState, inputType = BuildingSystems.Fluid.Types.InputType.Stages, m_flow_nominal = 0.2) annotation(
      Placement(transformation(origin = {84, -66}, extent = {{-10, -10}, {10, 10}})));
  equation
    der(EGrid) = battery.PGrid;
    der(EPVField) = pvField.PField;
    connect(radiation.radiationPort, pvField.radiationPort) annotation(
      Line(points = {{-36, 8}, {-28, 8}, {-28, -13.8}}, color = {255, 255, 0}));
    connect(ambience.toSurfacePorts, building.toAmbienceSurfacesPorts) annotation(
      Line(points = {{101, 30}, {107, 30}}, color = {0, 255, 0}));
    connect(ambience.toAirPorts, building.toAmbienceAirPorts) annotation(
      Line(points = {{101, 22}, {107, 22}}, color = {85, 170, 255}));
    connect(building.airchange[1], airchange.y) annotation(
      Line(points = {{125.8, 30}, {131.8, 30}}, color = {0, 0, 127}));
    connect(ambience.TAirRef, building.TAirAmb) annotation(
      Line(points = {{83, 33}, {80, 33}, {80, 42}, {122, 42}, {122, 36}}, color = {0, 0, 127}));
    connect(ambience.xAir, building.xAirAmb) annotation(
      Line(points = {{83, 31}, {78, 31}, {78, 46}, {124, 46}, {124, 36}}, color = {0, 0, 127}));
    connect(ambience.IrrDirHor, radiation.IrrDirHor) annotation(
      Line(points = {{83, 29}, {-56, 29}, {-56, 14}, {-51.6, 14}}, color = {0, 0, 127}));
    connect(radiation.IrrDifHor, ambience.IrrDifHor) annotation(
      Line(points = {{-51.6, 10}, {-58, 10}, {-58, 28}, {83, 28}, {83, 27}}, color = {0, 0, 127}));
    connect(ambience.TAirRef, pvField.TAmb) annotation(
      Line(points = {{83, 33}, {-24, 33}, {-24, -14}}, color = {0, 0, 127}));
    connect(pvField.angleDegTil, radiation.angleDegTil) annotation(
      Line(points = {{-32, -14}, {-58, -14}, {-58, 6}, {-51.6, 6}}, color = {0, 0, 127}));
    connect(pvField.angleDegAzi, radiation.angleDegAzi) annotation(
      Line(points = {{-32, -16}, {-32, -16}, {-58, -16}, {-58, 2}, {-51.6, 2}}, color = {0, 0, 127}));
    connect(ambience.latitudeDeg, radiation.latitudeDeg) annotation(
      Line(points = {{85, 35}, {85, 50}, {-47.8, 50}, {-47.8, 15.6}}, color = {0, 0, 127}));
    connect(ambience.longitudeDeg, radiation.longitudeDeg) annotation(
      Line(points = {{87, 35}, {87, 50}, {-44, 50}, {-44, 15.6}}, color = {0, 0, 127}));
    connect(ambience.longitudeDeg0, radiation.longitudeDeg0) annotation(
      Line(points = {{89, 35}, {89, 50}, {-40, 50}, {-40, 15.6}}, color = {0, 0, 127}));
    connect(TSet.y, heatPump.TSet) annotation(
      Line(points = {{175.6, -114}, {175.6, -123}, {143.6, -123}}, color = {0, 0, 127}));
    connect(expHP.port_a, heatPump.port_a1) annotation(
      Line(points = {{202, -102}, {202, -126}, {142, -126}}, color = {0, 127, 255}));
    connect(tempHP.port_b, expHP.port_a) annotation(
      Line(points = {{172, -102}, {202, -102}}, color = {0, 127, 255}));
    connect(tempRadOUT.port_a, radiator.port_b) annotation(
      Line(points = {{202, -26}, {198, -26}}, color = {0, 127, 255}));
    connect(heatPump.port_a2, boundary.ports[1]) annotation(
      Line(points = {{122, -138}, {113, -138}, {113, -150}, {98, -150}}, color = {0, 127, 255}));
    connect(heatPump.port_b2, boundary2.ports[1]) annotation(
      Line(points = {{142, -138}, {154, -138}, {154, -148}, {172, -148}}, color = {0, 127, 255}));
    connect(storage.port_HX_1_b, tempHP.port_a) annotation(
      Line(points = {{129, -68}, {108, -68}, {108, -102}, {152, -102}}, color = {0, 127, 255}));
    connect(radiator.port_a, tempRadIN.port_b) annotation(
      Line(points = {{178, -26}, {172, -26}}, color = {0, 127, 255}));
    connect(tempStorageOUT.port_a, pumpRad.port_a) annotation(
      Line(points = {{100, -40}, {94, -40}, {94, -26}, {124, -26}}, color = {0, 127, 255}));
    connect(pumpRad.port_b, tempRadIN.port_a) annotation(
      Line(points = {{144, -26}, {152, -26}}, color = {0, 127, 255}));
    connect(hysRad1.y, not11.u) annotation(
      Line(points = {{34.6, -32}, {42.6, -32}}, color = {255, 0, 255}));
    connect(not11.y, booToInt1.u) annotation(
      Line(points = {{56.6, -32}, {56.6, -33}, {62.6, -33}}, color = {255, 0, 255}));
    connect(storage.port_HX_1_a, pumpHP.port_b) annotation(
      Line(points = {{129, -66}, {94, -66}}, color = {0, 127, 255}));
    connect(pumpHP.port_a, heatPump.port_b1) annotation(
      Line(points = {{74, -66}, {48, -66}, {48, -126}, {122, -126}}, color = {0, 127, 255}));
    connect(storage.port_b1, tempStorageOUT.port_b) annotation(
      Line(points = {{143, -53}, {118, -53}, {118, -40}, {120, -40}}, color = {0, 127, 255}));
    connect(storage.port_a1, tempRadOUT.port_b) annotation(
      Line(points = {{143, -71}, {230, -71}, {230, -26}, {222, -26}}, color = {0, 127, 255}));
    connect(expRad.port_a, storage.port_a1) annotation(
      Line(points = {{240, -78}, {145, -78}, {145, -71}, {143, -71}}, color = {0, 127, 255}));
    connect(booToInt1.y, pumpHP.stage) annotation(
      Line(points = {{74.5, -33}, {84, -33}, {84, -54}}, color = {255, 127, 0}));
    connect(heatPump.P, battery.PLoad) annotation(
      Line(points = {{121, -132}, {-4, -132}, {-4, -72}, {-13, -72}}, color = {0, 0, 127}));
    connect(ambience.TAirRef, boundary.T_in) annotation(
      Line(points = {{83, 33}, {2, 33}, {2, -146}, {76, -146}}, color = {0, 0, 127}));
  connect(not1.u, hysRad.y) annotation(
      Line(points = {{96.8, -2}, {86.8, -2}}, color = {255, 0, 255}));
  connect(not1.y, booToInt.u) annotation(
      Line(points = {{110.6, -2}, {117.6, -2}}, color = {255, 0, 255}));
  connect(booToInt.y, pumpRad.stage) annotation(
      Line(points = {{130.6, -2}, {133.6, -2}, {133.6, -14}}, color = {255, 127, 0}));
  connect(storage.T[1], hysRad1.u) annotation(
      Line(points = {{143, -56}, {149.4, -56}, {149.4, -86}, {13.4, -86}, {13.4, -32}, {19.4, -32}}, color = {0, 0, 127}));
  connect(hysRad.u, building.TAir[1]) annotation(
      Line(points = {{72, -2}, {66, -2}, {66, 12}, {140, 12}, {140, 20}, {136, 20}}, color = {0, 0, 127}));
  connect(building.radHeatSourcesPorts[1], radiator.heatPortCon) annotation(
      Line(points = {{118, 36}, {118, 48}, {186, 48}, {186, -18}}, color = {191, 0, 0}));
  connect(radiator.heatPortRad, building.conHeatSourcesPorts[1]) annotation(
      Line(points = {{190, -18}, {190, 52}, {116, 52}, {116, 36}}, color = {191, 0, 0}));
  connect(pvField.PField, battery.PCharge) annotation(
      Line(points = {{-20, -14}, {-14, -14}, {-14, -56}, {-34, -56}, {-34, -72}, {-22, -72}}, color = {0, 0, 127}));
    annotation(
      experiment(StartTime = 0, StopTime = 31536000),
      __Dymola_Commands(file = "modelica://BuildingSystems/Resources/Scripts/Dymola/Applications/AirConditioningSystems/PhotovoltaicCoolingSystem.mos" "Simulate and plot"),
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-60, 60}, {260, -160}})),
      Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}})),
      Documentation(info = "<html>
  <p>
  Example that simulates a photovoltaic driven air-conditioning system:
  The PV generator loads an electric battery. The compression chiller takes
  the electricity from the battery, produces cold water and charges it
  into a cold water storage. The cold water is used for the air-conditioning of a building.
  </p>
  </html>", revisions = "<html>
  <ul>
  <li>
  August 18, 2018, by Christoph Nytsch-Geusen:<br/>
  Adapted to possible different media for the storage content and the two internal heat exchangers.
  </li>
  <li>
  May 21, 2016, by Christoph Nytsch-Geusen:<br/>
  First implementation.
  </li>
  </ul>
  </html>"),
      uses(BuildingSystems(version = "2.0.0-beta")),
      version = "");
  end CommunitySystemOnGrid;

  model CommunitySystemOnGridTest "Photovoltaic cooling system"
    extends Modelica.Icons.Example;
    package Medium1 = BuildingSystems.Media.Water;
    package Medium2 = BuildingSystems.Media.Air;
    parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal = -5 "Temperature difference evaporator inlet-outlet";
    parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal = 10 "Temperature difference condenser outlet-inlet";
    parameter Modelica.Units.SI.HeatFlowRate QCon_flow_nominal = 100E3 "Evaporator heat flow rate";
    parameter Modelica.Units.SI.MassFlowRate m1_flow_nominal = QCon_flow_nominal/dTCon_nominal/4200 "Nominal mass flow rate at condenser";
    parameter Modelica.Units.SI.MassFlowRate m_flow_nominal = 0.5 "Nominal mass flow rate";
    parameter Modelica.Units.SI.MassFlowRate m_flow = 0.5 "Mass flow rate in the cold water production and consumption loop";
    BuildingSystems.Technologies.Photovoltaics.PVModules.PVModuleSimpleMPP pvField(redeclare BuildingSystems.Technologies.Photovoltaics.Data.PhotovoltaicModules.TSM230PC05 pvModuleData, angleDegTil_constant = 30.0, angleDegAzi_constant = 0.0, nModPar = 8, nModSer = 10) annotation(
      Placement(transformation(extent = {{-54, -18}, {-34, 2}})));
    BuildingSystems.Climate.SolarRadiationTransformers.SolarRadiationTransformerIsotropicSky radiation(rhoAmb = 0.2, angleDegL = 0.0) annotation(
      Placement(transformation(extent = {{-72, 12}, {-52, 32}})));
    BuildingSystems.Technologies.ElectricalStorages.BatterySimple battery(redeclare BuildingSystems.Technologies.ElectricalStorages.Data.LeadAcid.LeadAcidGeneric batteryData, nBat = 3) annotation(
      Placement(transformation(origin = {6, -74}, extent = {{-38, -10}, {-18, 10}})));
    BuildingSystems.Buildings.BuildingTemplates.Building1Zone1DDistrict building(calcIdealLoads = false, heatSources = false, angleDegAziBuilding = 0.0, width = 10, length = 14, heightSto = 2.8, nSto = 3, ARoom = 70, widthWindow1 = 6, heightWindow1 = 2*1.2, widthWindow2 = 6, heightWindow2 = 2*1.2, widthWindow3 = 6, heightWindow3 = 2*1.2, widthWindow4 = 6, heightWindow4 = 2*1.2, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.OuterWallMultistorey1958to1968 constructionWall1, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.OuterWallMultistorey1958to1968 constructionWall2, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.OuterWallMultistorey1958to1968 constructionWall3, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.OuterWallMultistorey1958to1968 constructionWall4, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.BasePlateMultistorey1958to1968 constructionBottom, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.IntermediateWallMultistorey1958to1968 constructionWallsInterior, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.IntermediateCeilingMultistorey1958to1968 constructionCeilingsInterior, redeclare BuildingSystems.Buildings.Data.Constructions.Transparent.SingleGlazing constructionWindow2, redeclare BuildingSystems.Buildings.Data.Constructions.Transparent.SingleGlazing constructionWindow3, redeclare BuildingSystems.Buildings.Data.Constructions.Transparent.SingleGlazing constructionWindow4, redeclare BuildingSystems.Buildings.Data.Constructions.Transparent.SingleGlazing constructionWindow1, redeclare BuildingSystems.Buildings.Data.Constructions.Thermal.RoofRowhouse1918 constructionCeiling, BCWall1 = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, BCWall2 = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, BCWall3 = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, BCWall4 = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, BCCeiling = BuildingSystems.Buildings.Types.ThermalBoundaryCondition.Ambience, nHeatSources = 1) "6 bytů. každý 70m^2" annotation(
      Placement(transformation(extent = {{88, 30}, {108, 50}})));
    BuildingSystems.Buildings.Ambience ambience(nSurfaces = building.nSurfacesAmbience, redeclare block WeatherData = BuildingSystems.Climate.WeatherDataMeteonorm.Germany_Berlin_Meteonorm_ASCII) "Ambience model" annotation(
      Placement(transformation(extent = {{64, 30}, {84, 50}})));
    Modelica.Blocks.Sources.Constant airchange(k = 1.5) "větrání, kolikrát za hodinu se vymění objem vzducuhu v budově" annotation(
      Placement(transformation(origin = {116, 44}, extent = {{-2, -2}, {2, 2}}, rotation = 180)));
    Modelica.Units.SI.Energy EGrid(start = 0.0) "Integrates the electricity taken from the grid";
    Modelica.Units.SI.Energy EPVField(start = 0.0) "Integrates the electricity generated by the PV field";
    BuildingSystems.Fluid.Sources.Boundary_pT bou1(redeclare package Medium = Medium2, nPorts = 1) annotation(
      Placement(transformation(origin = {108, -80}, extent = {{6, -6}, {-6, 6}})));
    BuildingSystems.Fluid.Sources.MassFlowSource_T boundary(redeclare package Medium = Medium2, m_flow = 0.5, nPorts = 1, use_T_in = true) annotation(
      Placement(transformation(origin = {20, -92}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.HeatPumps.Carnot_TCon heatPump(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, QCon_flow_nominal = 5000, allowFlowReversal1 = false, allowFlowReversal2 = false, dTCon_nominal = dTCon_nominal, dTEva_nominal = dTEva_nominal, dp1_nominal = 1000, dp2_nominal = 1000, show_T = true) annotation(
      Placement(transformation(origin = {70, -74}, extent = {{10, -10}, {-10, 10}})));
    BuildingSystems.Fluid.Movers.FlowControlled_dp pump1(redeclare package Medium = Medium1, addPowerToMedium = false, m_flow_nominal = 0.1, nominalValuesDefineDefaultPressureCurve = true) annotation(
      Placement(transformation(origin = {20, -26}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant dpSet(k = 150000.0) annotation(
      Placement(transformation(origin = {254, -22}, extent = {{-204, 18}, {-216, 30}})));
    BuildingSystems.Technologies.ThermalStorages.FluidStorage storage(HX_2 = false, redeclare BuildingSystems.Technologies.ThermalStorages.BaseClasses.BuoyancyModels.Buoyancy1 HeatBuoyancy, redeclare package Medium = Medium1, redeclare package Medium_HX_1 = Medium1, redeclare package Medium_HX_2 = Medium1, height = 2, thickness_wall = 0.1, HX_1 = true, UA_HX_1 = 10000, V = 10, nEle = 10) annotation(
      Placement(transformation(origin = {76, -20}, extent = {{10, -10}, {-10, 10}})));
    Modelica.Blocks.Sources.Constant TSet(k = 273.15 + 60.0) annotation(
      Placement(transformation(origin = {62, 56}, extent = {{44, -112}, {36, -104}})));
    BuildingSystems.Fluid.Storage.ExpansionVessel exp(redeclare package Medium = Medium1, V_start = 2, p_start = 2e5, T_start = 283.15) annotation(
      Placement(transformation(origin = {164, -58}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort temp(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {122, -28}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Storage.ExpansionVessel exp1(redeclare package Medium = Medium1, V_start = 0.1, p_start = 1.5e5) annotation(
      Placement(transformation(origin = {254, 16}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Movers.FlowControlled_dp pump11(redeclare package Medium = Medium1, addPowerToMedium = false, m_flow_nominal = 0.1, nominalValuesDefineDefaultPressureCurve = true, p_start = 150000) annotation(
      Placement(transformation(origin = {142, 20}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant dpSet1(k = 150000.0) annotation(
      Placement(transformation(origin = {380, 26}, extent = {{-204, 18}, {-216, 30}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort radTemp(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {220, 20}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {110, 10}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort tempinHX(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {44, -26}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort senTemIN(redeclare package Medium = Medium1, m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {170, 20}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.HeatExchangers.ConstantEffectiveness hex(redeclare package Medium1 = BuildingSystems.Media.Air "Moist air", redeclare package Medium2 = BuildingSystems.Media.Water "Water", m1_flow_nominal = 5, m2_flow_nominal = 6.12, dp1_nominal = 100, dp2_nominal = 1000, redeclare package Medium = BuildingSystems.Media.Water "Water") annotation(
      Placement(transformation(origin = {196, 26}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    BuildingSystems.Fluid.Sources.Boundary_pT bou(nPorts = 1, redeclare package Medium2 = Medium2, redeclare package Medium = BuildingSystems.Media.Air "Moist air") annotation(
      Placement(transformation(origin = {134, 80}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sources.MassFlowSource_T boundary1(nPorts = 1, redeclare package Medium2 = Medium2, redeclare package Medium = BuildingSystems.Media.Air "Moist air", m_flow = 2, T = 293.15) annotation(
      Placement(transformation(origin = {250, 80}, extent = {{10, -10}, {-10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort TempRoomIN(redeclare package Medium = BuildingSystems.Media.Air "Moist air", m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {222, 80}, extent = {{-10, -10}, {10, 10}})));
    BuildingSystems.Fluid.Sensors.TemperatureTwoPort TempRoomOUT(redeclare package Medium = BuildingSystems.Media.Air "Moist air", m_flow_nominal = 0.1) annotation(
      Placement(transformation(origin = {166, 80}, extent = {{-10, -10}, {10, 10}})));
  equation
    der(EGrid) = battery.PGrid;
    der(EPVField) = pvField.PField;
    connect(radiation.radiationPort, pvField.radiationPort) annotation(
      Line(points = {{-54, 22}, {-44, 22}, {-44, 0}}, color = {255, 255, 0}, smooth = Smooth.None));
    connect(pvField.PField, battery.PCharge) annotation(
      Line(points = {{-38, 0}, {-35.5, 0}, {-35.5, -74}, {-27, -74}}, color = {0, 0, 127}));
    connect(ambience.toSurfacePorts, building.toAmbienceSurfacesPorts) annotation(
      Line(points = {{83, 44}, {89, 44}}, color = {0, 255, 0}));
    connect(ambience.toAirPorts, building.toAmbienceAirPorts) annotation(
      Line(points = {{83, 36}, {89, 36}}, color = {85, 170, 255}));
    connect(building.airchange[1], airchange.y) annotation(
      Line(points = {{107.8, 44}, {113.8, 44}}, color = {0, 0, 127}));
    connect(ambience.TAirRef, building.TAirAmb) annotation(
      Line(points = {{65, 47}, {62, 47}, {62, 56}, {104.2, 56}, {104.2, 49.8}}, color = {0, 0, 127}));
    connect(ambience.xAir, building.xAirAmb) annotation(
      Line(points = {{65, 45}, {60, 45}, {60, 60}, {106.4, 60}, {106.4, 49.8}}, color = {0, 0, 127}));
    connect(ambience.IrrDirHor, radiation.IrrDirHor) annotation(
      Line(points = {{65, 43}, {-74, 43}, {-74, 28}, {-69.6, 28}}, color = {0, 0, 127}));
    connect(radiation.IrrDifHor, ambience.IrrDifHor) annotation(
      Line(points = {{-69.6, 24}, {-76, 24}, {-76, 42}, {65, 42}, {65, 41}}, color = {0, 0, 127}));
    connect(ambience.TAirRef, pvField.TAmb) annotation(
      Line(points = {{65, 47}, {-42, 47}, {-42, 0}}, color = {0, 0, 127}));
    connect(pvField.angleDegTil, radiation.angleDegTil) annotation(
      Line(points = {{-50, 0}, {-76, 0}, {-76, 20}, {-69.6, 20}}, color = {0, 0, 127}));
    connect(pvField.angleDegAzi, radiation.angleDegAzi) annotation(
      Line(points = {{-50, -2}, {-50, -2}, {-76, -2}, {-76, 16}, {-69.6, 16}}, color = {0, 0, 127}));
    connect(ambience.latitudeDeg, radiation.latitudeDeg) annotation(
      Line(points = {{67, 49}, {67, 64}, {-65.8, 64}, {-65.8, 29.6}}, color = {0, 0, 127}));
    connect(ambience.longitudeDeg, radiation.longitudeDeg) annotation(
      Line(points = {{69, 49}, {69, 64}, {-62, 64}, {-62, 29.6}}, color = {0, 0, 127}));
    connect(ambience.longitudeDeg0, radiation.longitudeDeg0) annotation(
      Line(points = {{71, 49}, {71, 64}, {-58, 64}, {-58, 29.6}}, color = {0, 0, 127}));
    connect(heatPump.port_b2, bou1.ports[1]) annotation(
      Line(points = {{80, -80}, {102, -80}}, color = {0, 127, 255}));
    connect(heatPump.port_a2, boundary.ports[1]) annotation(
      Line(points = {{60, -80}, {45, -80}, {45, -92}, {30, -92}}, color = {0, 127, 255}));
    connect(dpSet.y, pump1.dp_in) annotation(
      Line(points = {{37.4, 2}, {37.4, -14}, {20, -14}}, color = {0, 0, 127}));
    connect(heatPump.port_b1, pump1.port_a) annotation(
      Line(points = {{60, -68}, {8, -68}, {8, -26}, {10, -26}}, color = {0, 127, 255}));
    connect(temp.port_b, heatPump.port_a1) annotation(
      Line(points = {{132, -28}, {148, -28}, {148, -68}, {80, -68}}, color = {0, 127, 255}));
    connect(TSet.y, heatPump.TSet) annotation(
      Line(points = {{97.6, -52}, {82, -52}, {82, -65}}, color = {0, 0, 127}));
    connect(exp.port_a, heatPump.port_a1) annotation(
      Line(points = {{164, -68}, {80, -68}}, color = {0, 127, 255}));
    connect(dpSet1.y, pump11.dp_in) annotation(
      Line(points = {{163.4, 50}, {152, 50}, {152, 32}, {142, 32}}, color = {0, 0, 127}));
    connect(boundary.T_in, ambience.TAirRef) annotation(
      Line(points = {{8, -88}, {0, -88}, {0, 47}, {65, 47}}, color = {0, 0, 127}));
    connect(battery.PLoad, heatPump.P) annotation(
      Line(points = {{-17, -74}, {59, -74}}, color = {0, 0, 127}));
    connect(senTem.port_b, pump11.port_a) annotation(
      Line(points = {{120, 10}, {132, 10}, {132, 20}}, color = {0, 127, 255}));
    connect(storage.port_HX_1_b, temp.port_a) annotation(
      Line(points = {{69, -26}, {54, -26}, {54, -42}, {100, -42}, {100, -28}, {112, -28}}, color = {0, 127, 255}));
    connect(pump1.port_b, tempinHX.port_a) annotation(
      Line(points = {{30, -26}, {34, -26}}, color = {0, 127, 255}));
    connect(tempinHX.port_b, storage.port_HX_1_a) annotation(
      Line(points = {{54, -26}, {61, -26}, {61, -24}, {69, -24}}, color = {0, 127, 255}));
    connect(senTemIN.port_a, pump11.port_b) annotation(
      Line(points = {{160, 20}, {152, 20}}, color = {0, 127, 255}));
    connect(senTem.port_a, storage.port_a2) annotation(
      Line(points = {{100, 10}, {69, 10}, {69, -11}}, color = {0, 127, 255}));
    connect(storage.port_b2, radTemp.port_b) annotation(
      Line(points = {{69, -29}, {69, -36}, {94, -36}, {94, -10}, {230, -10}, {230, 20}}, color = {0, 127, 255}));
    connect(exp1.port_a, storage.port_b2) annotation(
      Line(points = {{254, 6}, {254, -29}, {69, -29}}, color = {0, 127, 255}));
    connect(hex.port_a2, senTemIN.port_b) annotation(
      Line(points = {{186, 20}, {180, 20}}, color = {0, 127, 255}));
    connect(hex.port_b2, radTemp.port_a) annotation(
      Line(points = {{206, 20}, {210, 20}}, color = {0, 127, 255}));
    connect(boundary1.ports[1], TempRoomIN.port_b) annotation(
      Line(points = {{240, 80}, {232, 80}}, color = {0, 127, 255}));
    connect(TempRoomIN.port_a, hex.port_a1) annotation(
      Line(points = {{212, 80}, {206, 80}, {206, 32}}, color = {0, 127, 255}));
    connect(bou.ports[1], TempRoomOUT.port_a) annotation(
      Line(points = {{144, 80}, {156, 80}}, color = {0, 127, 255}));
    connect(TempRoomOUT.port_b, hex.port_b1) annotation(
      Line(points = {{176, 80}, {186, 80}, {186, 32}}, color = {0, 127, 255}));
    annotation(
      experiment(StartTime = 5184000, StopTime = 15552000, __Dymola_NumberOfIntervals = 1500, __Dymola_Algorithm = "Dassl"),
      __Dymola_Commands(file = "modelica://BuildingSystems/Resources/Scripts/Dymola/Applications/AirConditioningSystems/PhotovoltaicCoolingSystem.mos" "Simulate and plot"),
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-80, 140}, {280, -100}})),
      Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}})),
      Documentation(info = "<html>
  <p>
  Example that simulates a photovoltaic driven air-conditioning system:
  The PV generator loads an electric battery. The compression chiller takes
  the electricity from the battery, produces cold water and charges it
  into a cold water storage. The cold water is used for the air-conditioning of a building.
  </p>
  </html>", revisions = "<html>
  <ul>
  <li>
  August 18, 2018, by Christoph Nytsch-Geusen:<br/>
  Adapted to possible different media for the storage content and the two internal heat exchangers.
  </li>
  <li>
  May 21, 2016, by Christoph Nytsch-Geusen:<br/>
  First implementation.
  </li>
  </ul>
  </html>"));
  end CommunitySystemOnGridTest;
  annotation(
    uses(Modelica(version = "4.0.0"), Buildings(version = "12.0.0"), BuildingSystems(version = "2.0.0-beta")));
end HeatSimulations;
