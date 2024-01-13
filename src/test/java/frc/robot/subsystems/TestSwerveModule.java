package frc.robot.subsystems;

import static frc.robot.utils.ModuleMap.MODULE_POSITION;
import static frc.robot.utils.TestUtils.refreshAkitData;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.CAN;
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

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    Logger.start();

    m_robotTime = new RobotTime();

    /* create the TalonFX */
    m_testModule =
        new SwerveModule(
            MODULE_POSITION.FRONT_LEFT,
            new TalonFX(CAN.frontLeftTurnMotor),
            new TalonFX(CAN.frontLeftDriveMotor),
            new CANcoder(CAN.frontLeftCanCoder),
            0.0);

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

    m_testModule.setTurnAngle(testAngle);
    Timer.delay(WAIT_TIME);

    assertEquals(testAngle, m_testModule.getTurnHeadingDeg(), DELTA);
  }

  @Disabled
  public void testModuleSpeed() {
    var testSpeed = 4.0;

    m_testModule.setDesiredState(new SwerveModuleState(testSpeed, new Rotation2d()), false);
    Timer.delay(WAIT_TIME);

    for (int i = 0; i < 25; i++) {
      m_robotTime.periodic();
      Timer.delay(WAIT_TIME);
      m_testModule.simulationPeriodic();
      refreshAkitData();
    }

    assertEquals(testSpeed, m_testModule.getDriveMps(), DELTA);
  }

  @Override
  public void close() throws Exception {
    /* destroy our TalonFX object */
    m_testModule.close();
  }
}
