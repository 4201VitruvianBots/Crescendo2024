package frc.robot;

import static frc.robot.simulation.SimConstants.kMotorResistance;
import static frc.robot.utils.TestUtils.refreshAkitData;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.SWERVE;
import frc.robot.utils.CtreUtils;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

public class TestTalonFxMotorSim implements AutoCloseable {
  static final double DELTA = 0.2; // acceptable deviation range
  static final double WAIT_TIME = 0.02;
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

    refreshAkitData();

    m_driveMotorSim.setInputVoltage(
        MathUtil.clamp(m_driveMotorSimState.getMotorVoltage(), -12, 12));
    m_turnMotorSim.setInputVoltage(MathUtil.clamp(m_turnMotorSimState.getMotorVoltage(), -12, 12));

    m_currentTime = Logger.getRealTimestamp() * 1.0e-6;
    double dt = m_currentTime - m_lastTime;
    m_driveMotorSim.update(dt);
    m_turnMotorSim.update(dt);

    m_turnMotorSimState.setSupplyVoltage(
        RobotController.getBatteryVoltage()
            - m_turnMotorSim.getCurrentDrawAmps() * kMotorResistance);
    m_driveMotorSimState.setSupplyVoltage(
        RobotController.getBatteryVoltage()
            - m_driveMotorSim.getCurrentDrawAmps() * kMotorResistance);

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
    var driveConfig = CtreUtils.generateDriveMotorConfig();
    driveConfig.Feedback.SensorToMechanismRatio = SWERVE.MODULE.kDriveMotorGearRatio;
    m_driveMotor.getConfigurator().apply(driveConfig);
    m_driveMotorSimState = m_driveMotor.getSimState();

    m_turnMotor = new TalonFX(1);
    var turnConfig = CtreUtils.generateTurnMotorConfig();
    turnConfig.Feedback.SensorToMechanismRatio = SWERVE.MODULE.kTurnMotorGearRatio;
    m_turnMotor.getConfigurator().apply(turnConfig);
    m_turnMotorSimState = m_turnMotor.getSimState();

    m_turnMotorSim =
        new DCMotorSim(
            //            LinearSystemId.createDCMotorSystem(0.00002, 0.00001),
            SWERVE.MODULE.kTurnGearbox, SWERVE.MODULE.kTurnMotorGearRatio, 0.00001);
    m_driveMotorSim =
        new DCMotorSim(
            //                    LinearSystemId.createDCMotorSystem(0.8, 0.6),
            LinearSystemId.createDCMotorSystem(0.02, 0.001),
            SWERVE.MODULE.kDriveGearbox,
            SWERVE.MODULE.kDriveMotorGearRatio);

    /* enable the robot */
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    RoboRioSim.resetData();
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

  @Disabled("Constants do not work without feedforward")
  public void testVelocityControl() {
    var testSpeedMps = 4;
    var testSpeedRps = testSpeedMps / (SWERVE.MODULE.kWheelDiameterMeters * Math.PI);
    var velocityRequest = new VelocityVoltage(0);
    var velocitySignal = m_driveMotor.getVelocity().clone();
    m_driveMotor.setControl(velocityRequest.withVelocity(testSpeedRps));
    Timer.delay(WAIT_TIME);

    var velPub = m_table.getDoubleTopic("velocity").publish();
    var batteryVPub = m_table.getDoubleTopic("batteryV").publish();
    var voltagePub = m_table.getDoubleTopic("voltage").publish();
    var currentPub = m_table.getDoubleTopic("current").publish();
    var idxPub = m_table.getIntegerTopic("idx").publish();
    var stopPub = false;
    var testVelRps = 0.0;
    var testVelMps = 0.0;
    m_lastTime = 0;
    for (int i = 0; i < 25; i++) {
      Timer.delay(WAIT_TIME);
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
      if (testVelMps > testSpeedRps && !stopPub) {
        idxPub.set(i);
        stopPub = true;
      }
      m_nt.flush();
    }

    assertEquals(testSpeedMps, testVelMps, DELTA);
    velPub.close();
    idxPub.close();
  }

  @Disabled("Constants do not work without feedforward")
  public void testPositionControl() {
    var testPositionDeg = 90.0;
    var testPositionRot = (testPositionDeg / 360.0) * SWERVE.MODULE.kTurnMotorGearRatio;
    var positionRequest = new PositionVoltage(0);
    var positionSignal = m_turnMotor.getPosition().clone();
    m_turnMotor.setControl(positionRequest.withPosition(testPositionRot));
    Timer.delay(WAIT_TIME);

    var posPub = m_table.getDoubleTopic("position").publish();
    var batteryVPub = m_table.getDoubleTopic("batteryV").publish();
    var voltagePub = m_table.getDoubleTopic("voltage").publish();
    var currentPub = m_table.getDoubleTopic("current").publish();
    var idxPub = m_table.getIntegerTopic("idx").publish();
    var stopPub = false;
    var testPosRot = 0.0;
    var testPosDeg = 0.0;
    m_lastTime = 0;
    for (int i = 0; i < 25; i++) {
      Timer.delay(WAIT_TIME);
      simulateMotorModels();
      positionSignal.refresh();
      testPosRot = positionSignal.getValue();
      testPosDeg = testPosRot * 360.0 / SWERVE.MODULE.kTurnMotorGearRatio;

      System.out.printf("Idx: %2d\tRotations: %5.2f\tDegrees: %6.2f\n", i, testPosRot, testPosDeg);
      var setpoint =
          Double.valueOf(m_turnMotor.getAppliedControl().getControlInfo().get("Position"));
      m_turnMotor.getClosedLoopError().refresh();
      var error = m_turnMotor.getClosedLoopError().getValue();
      System.out.printf(
          "Turn Pos: %5.2f\tSetpoint: %5.2f\tError: %5.2f\n", testPosRot, setpoint, error);

      posPub.set(testPosDeg);
      voltagePub.set(m_turnMotorSimState.getMotorVoltage());
      currentPub.set(m_turnMotorSimState.getSupplyCurrent());
      batteryVPub.set(RobotController.getBatteryVoltage());
      if (positionSignal.getValue() > testPositionRot && !stopPub) {
        idxPub.set(i);
        stopPub = true;
      }
      m_nt.flush();
    }

    assertEquals(testPositionDeg, testPosDeg, DELTA);
    posPub.close();
    idxPub.close();
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

  @AfterEach
  @Override
  public void close() throws Exception {
    /* destroy our TalonFX object */
    m_driveMotor.close();
    m_turnMotor.close();
    m_nt.close();
  }
}
