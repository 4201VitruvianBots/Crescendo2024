package frc.robot.subsystems;

import static frc.robot.utils.TestUtils.refreshAkitData;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.constants.CAN;
import frc.robot.constants.SWERVE;
import frc.robot.utils.TestUtils;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

public class TestSwerveModule implements AutoCloseable {
  static final double DELTA = 0.2; // acceptable deviation range
  static final double WAIT_TIME = 0.02;

  NetworkTableInstance m_nt;
  NetworkTable m_table;

  RobotTime m_robotTime;
  SwerveModule m_testModule;
  static DCMotorSim SteerMotor;
  static DCMotorSim DriveMotor;

  private void updateSimModule(SwerveModule module, double deltaTime, double supplyVoltage) {
    TalonFXSimState steerMotor = module.getSteerMotor().getSimState();
    TalonFXSimState driveMotor = module.getDriveMotor().getSimState();
    CANcoderSimState cancoder = module.getCANcoder().getSimState();

    Unmanaged.feedEnable(20);
    var result = steerMotor.setSupplyVoltage(supplyVoltage);
    driveMotor.setSupplyVoltage(supplyVoltage);
    cancoder.setSupplyVoltage(supplyVoltage);
    Unmanaged.feedEnable(20);
    TestUtils.refreshAkitData();

    var testVoltage = steerMotor.getMotorVoltage();

    SteerMotor.setInputVoltage(
        addFriction(steerMotor.getMotorVoltage(), SWERVE.MODULE.kFrictionVoltage));
    DriveMotor.setInputVoltage(
        addFriction(driveMotor.getMotorVoltage(), SWERVE.MODULE.kFrictionVoltage));

    SteerMotor.update(deltaTime);
    DriveMotor.update(deltaTime);
    TestUtils.refreshAkitData();

    var testPos = SteerMotor.getAngularPositionRotations();
    var testVel = SteerMotor.getAngularVelocityRPM();

    steerMotor.setRawRotorPosition(SteerMotor.getAngularPositionRotations() * SWERVE.MODULE.kTurnMotorGearRatio);
    steerMotor.setRotorVelocity(
            SteerMotor.getAngularVelocityRPM() / 60.0 * SWERVE.MODULE.kTurnMotorGearRatio);

    /* CANcoders see the mechanism, so don't account for the steer gearing */
    cancoder.setRawPosition(SteerMotor.getAngularPositionRotations());
    cancoder.setVelocity(SteerMotor.getAngularVelocityRPM() / 60.0);

    driveMotor.setRawRotorPosition(
        DriveMotor.getAngularPositionRotations() * SWERVE.MODULE.kDriveMotorGearRatio);
    driveMotor.setRotorVelocity(
        DriveMotor.getAngularVelocityRPM() / 60.0 * SWERVE.MODULE.kDriveMotorGearRatio);
  }

  protected double addFriction(double motorVoltage, double frictionVoltage) {
    if (Math.abs(motorVoltage) < frictionVoltage) {
      motorVoltage = 0.0;
    } else if (motorVoltage > 0.0) {
      motorVoltage -= frictionVoltage;
    } else {
      motorVoltage += frictionVoltage;
    }
    return motorVoltage;
  }

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    Logger.start();

    m_robotTime = new RobotTime();
    RobotTime.setTimeMode(RobotTime.TIME_MODE.UNITTEST);

    /* create the TalonFX */
    m_testModule = new SwerveModule(SWERVE.testConstants, CAN.rioCanbus);
    SteerMotor =
        new DCMotorSim(
            DCMotor.getFalcon500(1), SWERVE.MODULE.kTurnMotorGearRatio, SWERVE.MODULE.kTurnInertia);
    DriveMotor =
        new DCMotorSim(
            DCMotor.getFalcon500(1),
            SWERVE.MODULE.kDriveMotorGearRatio,
            SWERVE.MODULE.kDriveInertia);

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

  @Test
  public void testModuleAngles() {
    var testAngle = 90.0;

    var testState = new SwerveModuleState(SWERVE.kMaxSpeedMetersPerSecond * 0.11, Rotation2d.fromDegrees(testAngle));

    var startPos = m_testModule.getPosition(true).angle.getDegrees();

    m_testModule.apply(testState, SwerveModule.DriveRequestType.Velocity);

    var posPub = m_table.getDoubleTopic("position").publish();
    var batteryVPub = m_table.getDoubleTopic("batteryV").publish();
    var voltagePub = m_table.getDoubleTopic("voltage").publish();
    var currentPub = m_table.getDoubleTopic("current").publish();

    var startPos2 = m_testModule.getPosition(true).angle.getDegrees();

    for (int i = 0; i < 25; i++) {
      SimHooks.stepTiming(WAIT_TIME);
      m_robotTime.periodic();
      updateSimModule(m_testModule, RobotTime.getTimeDelta(), RobotController.getBatteryVoltage());
      posPub.set(m_testModule.getPosition(true).angle.getDegrees());
      voltagePub.set(m_testModule.getSteerMotor().getMotorVoltage().getValue());
      currentPub.set(m_testModule.getSteerMotor().getStatorCurrent().getValue());
      batteryVPub.set(RobotController.getBatteryVoltage());
      m_nt.flush();
    }

    var testPos = m_testModule.getPosition(true).angle.getDegrees();
    assertEquals(testAngle, m_testModule.getPosition(true).angle.getDegrees(), DELTA);
  }

  @Disabled
  public void testModuleSpeed() {
    //    var testSpeed = 4.0;
    //
    //    m_testModule.setDesiredState(new SwerveModuleState(testSpeed, new Rotation2d()), false);
    //    Timer.delay(WAIT_TIME);
    //
    //    for (int i = 0; i < 25; i++) {
    //      m_robotTime.periodic();
    //      Timer.delay(WAIT_TIME);
    //      m_testModule.simulationPeriodic();
    //      refreshAkitData();
    //    }
    //
    //    assertEquals(testSpeed, m_testModule.getDriveMps(), DELTA);
  }

  @Override
  public void close() throws Exception {
    /* destroy our TalonFX object */
    //    m_testModule.close();
  }
}
