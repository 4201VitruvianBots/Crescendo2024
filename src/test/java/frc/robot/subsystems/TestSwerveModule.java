package frc.robot.subsystems;

import static frc.robot.simulation.SimConstants.kMotorResistance;
import static frc.robot.utils.TestUtils.refreshAkitData;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.*;
import frc.robot.constants.CAN;
import frc.robot.constants.SWERVE;
import frc.robot.utils.TestUtils;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

@Disabled("Not working")
public class TestSwerveModule implements AutoCloseable {
  static final double DELTA = 0.2; // acceptable deviation range
  static final double WAIT_TIME = 0.04;

  NetworkTableInstance m_nt;
  NetworkTable m_table;

  SwerveModule m_testModule;

  private DCMotorSim m_turnMotorModel;
  private DCMotorSim m_driveMotorModel;
  private TalonFXSimState m_turnSimState;
  private TalonFXSimState m_driveSimState;
  private CANcoderSimState m_canCoderSimSate;

  private final TalonFXConfiguration m_testTurnConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration m_testDriveConfig = new TalonFXConfiguration();

  private void updateSimModule() {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_driveMotorModel.getCurrentDrawAmps(), m_turnMotorModel.getCurrentDrawAmps()));

    m_turnSimState.setSupplyVoltage(
        RobotController.getBatteryVoltage()
            - m_turnMotorModel.getCurrentDrawAmps() * kMotorResistance);
    m_driveSimState.setSupplyVoltage(
        RobotController.getBatteryVoltage()
            - m_driveMotorModel.getCurrentDrawAmps() * kMotorResistance);
    m_canCoderSimSate.setSupplyVoltage(RobotController.getBatteryVoltage());
    TestUtils.refreshAkitData();
    Timer.delay(WAIT_TIME);

    var testTurnMotorVoltage = m_turnSimState.getMotorVoltage();
    var testDriveMotorVoltage = m_driveSimState.getMotorVoltage();

    m_turnMotorModel.setInputVoltage(
        addFriction(m_turnSimState.getMotorVoltage(), SWERVE.MODULE.kFrictionVoltage));
    m_driveMotorModel.setInputVoltage(
        addFriction(m_driveSimState.getMotorVoltage(), SWERVE.MODULE.kFrictionVoltage));

    m_turnMotorModel.update(0.02);
    m_driveMotorModel.update(0.02);

    m_turnSimState.setRawRotorPosition(
        m_turnMotorModel.getAngularPositionRotations() * SWERVE.MODULE.kTurnMotorGearRatio);
    m_turnSimState.setRotorVelocity(
        m_turnMotorModel.getAngularVelocityRPM() / 60.0 * SWERVE.MODULE.kTurnMotorGearRatio);

    /* CANcoders see the mechanism, so don't account for the steer gearing */
    m_canCoderSimSate.setRawPosition(m_turnMotorModel.getAngularPositionRotations());
    m_canCoderSimSate.setVelocity(m_turnMotorModel.getAngularVelocityRPM() / 60.0);

    m_driveSimState.setRawRotorPosition(
        m_driveMotorModel.getAngularPositionRotations() * SWERVE.MODULE.kDriveMotorGearRatio);
    m_driveSimState.setRotorVelocity(
        m_driveMotorModel.getAngularVelocityRPM() / 60.0 * SWERVE.MODULE.kDriveMotorGearRatio);
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

    /* create the TalonFX */
    m_testModule = new SwerveModule(SWERVE.FrontLeftConstants, CAN.canivoreCanbus);
    m_turnMotorModel =
        new DCMotorSim(
            DCMotor.getFalcon500(1), SWERVE.MODULE.kTurnMotorGearRatio, SWERVE.MODULE.kTurnInertia);
    m_driveMotorModel =
        new DCMotorSim(
            DCMotor.getFalcon500(1),
            SWERVE.MODULE.kDriveMotorGearRatio,
            SWERVE.MODULE.kDriveInertia);

    m_turnSimState = m_testModule.getSteerMotor().getSimState();
    m_turnSimState.Orientation =
        SWERVE.MODULE.kTurnInverted
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    m_driveSimState = m_testModule.getDriveMotor().getSimState();
    m_canCoderSimSate = m_testModule.getCANcoder().getSimState();

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
    Timer.delay(0.300);
  }

  @AfterEach
  void shutdown() throws Exception {
    close();
  }

  @Test
  public void testModuleAngles() {
    var testAngle = 0.0;

    var testState =
        new SwerveModuleState(
            SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.11, Rotation2d.fromDegrees(testAngle));

    m_testModule.apply(testState, SwerveModule.DriveRequestType.Velocity);
    refreshAkitData();
    Timer.delay(WAIT_TIME);

    var posPub = m_table.getDoubleTopic("position").publish();
    var voltagePub = m_table.getDoubleTopic("voltage").publish();
    var currentPub = m_table.getDoubleTopic("current").publish();

    for (int i = 0; i < 25; i++) {
      updateSimModule();
      posPub.set(m_testModule.getPosition(true).angle.getDegrees());
      voltagePub.set(m_testModule.getSteerMotor().getMotorVoltage().getValue());
      currentPub.set(m_testModule.getSteerMotor().getStatorCurrent().getValue());
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
