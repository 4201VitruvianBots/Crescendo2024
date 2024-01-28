package frc.robot;

import static frc.robot.simulation.SimConstants.kMotorResistance;
import static frc.robot.utils.TestUtils.refreshAkitData;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.*;
import frc.robot.constants.SWERVE;
import frc.robot.utils.CtreUtils;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

public class TestTalonFxMotorSim implements AutoCloseable {
  static final double DELTA = 0.2; // acceptable deviation range
  static final double WAIT_TIME = 0.02; // WAIT_TIME MUST BE ~0.02 FOR VALUES TO UPDATE PROPERLY
  static final double MAX_SPEED_RPS = 6380.0 / 60.0;

  NetworkTableInstance m_nt;
  NetworkTable m_table;

  TalonFX m_driveMotor;
  TalonFXSimState m_driveMotorSimState;
  TalonFX m_turnMotor;
  TalonFXSimState m_turnMotorSimState;

  private DCMotorSim m_turnMotorSim;
  private DCMotorSim m_driveMotorSim;

  double m_currentTime = 0;
  double m_lastTime = 0;

  private void simulateMotorModels() {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_driveMotorSim.getCurrentDrawAmps(), m_turnMotorSim.getCurrentDrawAmps()));

    m_turnMotorSimState.setSupplyVoltage(
        RobotController.getBatteryVoltage()
            - m_turnMotorSim.getCurrentDrawAmps() * kMotorResistance);
    m_driveMotorSimState.setSupplyVoltage(
        RobotController.getBatteryVoltage()
            - m_driveMotorSim.getCurrentDrawAmps() * kMotorResistance);
    refreshAkitData();
    Timer.delay(WAIT_TIME);

    m_driveMotorSim.setInputVoltage(
        MathUtil.clamp(m_driveMotorSimState.getMotorVoltage(), -12, 12));
    m_turnMotorSim.setInputVoltage(MathUtil.clamp(m_turnMotorSimState.getMotorVoltage(), -12, 12));

    m_currentTime = Logger.getRealTimestamp() * 1.0e-6;
    double dt = m_currentTime - m_lastTime;
    m_driveMotorSim.update(dt);
    m_turnMotorSim.update(dt);

    //    System.out.printf("DT: %.2f\tDrive Rotations: %.2f\tDrive RPM: %.2f\n", dt,
    // m_driveMotorSim.getAngularPositionRotations(), m_driveMotorSim.getAngularVelocityRPM());

    m_driveMotorSimState.setRawRotorPosition(
        m_driveMotorSim.getAngularPositionRotations() * SWERVE.MODULE.kDriveMotorGearRatio);
    m_driveMotorSimState.setRotorVelocity(
        m_driveMotorSim.getAngularVelocityRPM() * SWERVE.MODULE.kDriveMotorGearRatio / 60.0);
    m_turnMotorSimState.setRawRotorPosition(
        m_turnMotorSim.getAngularPositionRotations() * SWERVE.MODULE.kTurnMotorGearRatio);
    m_turnMotorSimState.setRotorVelocity(
        m_turnMotorSim.getAngularVelocityRPM() * SWERVE.MODULE.kTurnMotorGearRatio / 60.0);

    m_lastTime = m_currentTime;
  }

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    Logger.start();

    /* create the TalonFX */
    m_driveMotor = new TalonFX(0);
    // Don't know why unit test requires different constants than simulation/pre-generated PID Constants
    var driveConfig = CtreUtils.generateDriveMotorConfig();
    driveConfig.Slot0.kP = 0.5;
    driveConfig.Slot0.kV = 0.7;
    CtreUtils.configureTalonFx(m_driveMotor, driveConfig);
    m_driveMotorSimState = m_driveMotor.getSimState();

    m_turnMotor = new TalonFX(1);
    CtreUtils.configureTalonFx(m_turnMotor, CtreUtils.generateTurnMotorConfig());
    m_turnMotorSimState = m_turnMotor.getSimState();

    m_turnMotorSim =
        new DCMotorSim(
            SWERVE.MODULE.kTurnGearbox,
            SWERVE.MODULE.kTurnMotorGearRatio,
            SWERVE.MODULE.kTurnInertia);
    m_driveMotorSim =
        new DCMotorSim(
            SWERVE.MODULE.kDriveGearbox,
            SWERVE.MODULE.kDriveMotorGearRatio,
            SWERVE.MODULE.kDriveInertia);

    /* enable the robot */
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    RoboRioSim.resetData();
    SimHooks.setProgramStarted();
    refreshAkitData();

    m_nt = NetworkTableInstance.getDefault();
    m_nt.setServer("localhost", NetworkTableInstance.kDefaultPort4 + 1);
    m_nt.startClient4("unittest");
    m_table = m_nt.getTable("unittest");

    /* delay ~100ms so the devices can start up and enable */
    Timer.delay(0.200);
  }

  @AfterEach
  void shutdown() throws Exception {
    close();
  }

  @Test
  public void testVelocityControl() {
    var testSpeedMps = 4;
    var testSpeedRps = testSpeedMps / (SWERVE.MODULE.kWheelDiameterMeters * Math.PI);
    var velocityRequest = new VelocityVoltage(0);
    var velocitySignal = m_driveMotor.getVelocity().clone();
    m_driveMotor.setControl(velocityRequest.withVelocity(testSpeedRps));
    refreshAkitData();
    Timer.delay(WAIT_TIME);

    var velPub = m_table.getDoubleTopic("velocity").publish();
    var batteryVPub = m_table.getDoubleTopic("batteryV").publish();
    var voltagePub = m_table.getDoubleTopic("voltage").publish();
    var currentPub = m_table.getDoubleTopic("current").publish();
    var testVelRps = 0.0;
    var testVelMps = 0.0;
    m_lastTime = Logger.getRealTimestamp() * 1e-6;
    for (int i = 0; i < 25; i++) {
      simulateMotorModels();
      velocitySignal.refresh();
      testVelRps = m_driveMotor.getVelocity().getValue();
      testVelMps = testVelRps * (Math.PI * SWERVE.MODULE.kWheelDiameterMeters);

      System.out.printf("Idx: %2d\tRPS: %.2f\tMPS: %.2f\n", i, testVelRps, testVelMps);
      var setpoint =
          Double.valueOf(m_driveMotor.getAppliedControl().getControlInfo().get("Velocity"));
      System.out.printf(
          "Setpoint RPS: %5.2f\tMPS: %6.2f\n",
          setpoint, setpoint * (Math.PI * SWERVE.MODULE.kWheelDiameterMeters));
      m_driveMotor.getClosedLoopError().refresh();
      var error = m_driveMotor.getClosedLoopError().getValue();
      System.out.printf(
          "Error RPS: %5.2f\tMPS: %6.2f\n",
          error, error * (Math.PI * SWERVE.MODULE.kWheelDiameterMeters));

      velPub.set(testVelMps);
      voltagePub.set(m_driveMotorSimState.getMotorVoltage());
      currentPub.set(m_driveMotorSimState.getSupplyCurrent());
      batteryVPub.set(RobotController.getBatteryVoltage());
      m_nt.flush();
    }

    assertEquals(testSpeedMps, testVelMps, DELTA);
  }

  @Test
  public void testPositionControl() {
    var testPositionDeg = 90.0;
    var testPositionRot = (testPositionDeg / 360.0);
    var positionRequest = new PositionVoltage(0);
    var positionSignal = m_turnMotor.getPosition().clone();
    m_turnMotor.setControl(positionRequest.withPosition(testPositionRot));
    refreshAkitData();
    Timer.delay(WAIT_TIME);

    var posPub = m_table.getDoubleTopic("position").publish();
    var batteryVPub = m_table.getDoubleTopic("batteryV").publish();
    var voltagePub = m_table.getDoubleTopic("voltage").publish();
    var currentPub = m_table.getDoubleTopic("current").publish();
    var positionRot = 0.0;
    var positionDeg = 0.0;
    m_lastTime = Logger.getRealTimestamp() * 1e-6;
    for (int i = 0; i < 25; i++) {
      simulateMotorModels();
      positionSignal.refresh();
      positionRot = positionSignal.getValue();
      positionDeg = positionRot * 360.0;

      System.out.printf(
          "Idx: %2d\tRotations: %5.2f\tDegrees: %6.2f\n", i, positionRot, positionDeg);
      var setpoint =
          Double.valueOf(m_turnMotor.getAppliedControl().getControlInfo().get("Position"));
      m_turnMotor.getClosedLoopError().refresh();
      var error = m_turnMotor.getClosedLoopError().getValue();
      System.out.printf(
          "Turn Pos: %5.2f\tSetpoint: %5.2f\tError: %5.2f\n", positionRot, setpoint, error);

      posPub.set(positionDeg);
      voltagePub.set(m_turnMotorSimState.getMotorVoltage());
      currentPub.set(m_turnMotorSimState.getSupplyCurrent());
      batteryVPub.set(RobotController.getBatteryVoltage());

      m_nt.flush();
    }

    assertEquals(testPositionDeg, positionDeg, DELTA);
  }

  @Test
  public void testPositionSignal() {
    m_turnMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    refreshAkitData();

    var posReq = new PositionVoltage(0);
    var posSig = m_turnMotor.getPosition().clone();

    m_turnMotor.setControl(posReq.withPosition(5));
    Timer.delay(WAIT_TIME);
    posSig.refresh();

    var test = Double.valueOf(m_turnMotor.getAppliedControl().getControlInfo().get("Position"));

    assertNotEquals(0.0, test);
  }

  @Test
  public void testSimDevices() {
    assertEquals(12, BatterySim.calculateDefaultBatteryLoadedVoltage(0), 0.2);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(0));
    assertEquals(12, RobotController.getBatteryVoltage(), DELTA);
  }

  @Test
  public void testPIDError() {
    var motorA = new TalonFX(2);
    var motorAPosSignal = motorA.getPosition().clone();
    var motorAPosControl = new PositionVoltage(0);
    var motorAError = motorA.getClosedLoopError().clone();

    var motorB = new TalonFX(3);
    var motorBVelSignal = motorB.getVelocity().clone();
    var motorBVelControl = new VelocityVoltage(0);
    var motorBError = motorB.getClosedLoopError().clone();

    var config = CtreUtils.generateTurnMotorConfig();
    config.Feedback.SensorToMechanismRatio = SWERVE.MODULE.kTurnMotorGearRatio;
    motorA.getConfigurator().apply(config);
    Timer.delay(0.02);

    motorA.setControl(motorAPosControl.withPosition(5));
    Timer.delay(0.02);
    motorAPosSignal.refresh();
    motorAError.refresh();
    var motorASetpoint =
        Double.valueOf(motorA.getAppliedControl().getControlInfo().get("Position"));
    System.out.printf(
        "Motor A Pos: %5.2f\tSetpoint: %5.2f\tError: %5.2f\n",
        motorAPosSignal.getValue(), motorASetpoint, motorAError.getValue());

    motorB.setControl(motorBVelControl.withVelocity(2));
    Timer.delay(0.02);
    motorBVelSignal.refresh();
    motorBError.refresh();
    var motorBSetpoint =
        Double.valueOf(motorB.getAppliedControl().getControlInfo().get("Velocity"));
    System.out.printf(
        "Motor B Vel: %5.2f\tSetpoint: %5.2f\tError: %5.2f\n",
        motorBVelSignal.getValue(), motorBSetpoint, motorBError.getValue());

    System.out.println();
  }

  /** Example of running Physics Sim in AdvantageKit */
  @Test
  public void testTalonFXSim() {
    var testMotor = new TalonFX(4);
    var testConfig = new TalonFXConfiguration();
    testConfig.Slot0.kP = 100;
    var returnCode = testMotor.getConfigurator().apply(testConfig);

    var testMotorSimState = testMotor.getSimState();
    var velocitySignal = testMotor.getVelocity().clone();

    var velocityControl = new VelocityVoltage(0);
    testMotor.setControl(velocityControl.withVelocity(100));
    testMotor.setControl(new VoltageOut(12));

    // Delay and refreshAkitData() to update the TalonFX Control Properly THIS ORDER MATTERS
    refreshAkitData();
    Timer.delay(WAIT_TIME);

    var testInputVoltage =
        RobotController.getInputVoltage() - m_driveMotorSim.getCurrentDrawAmps() * 0.02;

    testMotorSimState.setSupplyVoltage(testInputVoltage);
    // Delay and refreshAkitData() to update supplyVoltage properly THIS ORDER MATTERS
    refreshAkitData();
    Timer.delay(WAIT_TIME);

    var testSimMotorVoltage = testMotorSimState.getMotorVoltage();

    assertNotEquals(0.0, testSimMotorVoltage);
    m_driveMotorSim.setInputVoltage(testSimMotorVoltage);

    m_driveMotorSim.update(0.02);

    testMotorSimState.setRawRotorPosition(m_driveMotorSim.getAngularPositionRotations());
    testMotorSimState.setRotorVelocity(m_driveMotorSim.getAngularVelocityRPM() / 60.0);

    velocitySignal.refresh();

    assertNotEquals(0.0, testSimMotorVoltage);
  }

  @AfterEach
  @Override
  public void close() throws Exception {
    /* destroy our TalonFX object */
    m_driveMotor.close();
    m_turnMotor.close();
    m_nt.close();
  }
}
