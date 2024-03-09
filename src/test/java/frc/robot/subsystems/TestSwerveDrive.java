package frc.robot.subsystems;

import static frc.robot.utils.TestUtils.refreshAkitData;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

public class TestSwerveDrive {
  static final double DELTA = 0.2; // acceptable deviation range
  static final double WAIT_TIME = 0.02;

  NetworkTableInstance m_nt;
  NetworkTable m_table;

  RobotTime m_robotTime;
  CommandSwerveDrivetrain m_swerveDrive;

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    Logger.start();

    m_robotTime = new RobotTime();

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

  @Test
  public void testChasissSpeedInvert() {
    var testChassisSpeeds = new ChassisSpeeds(1, 1, Math.PI);
    var invertedSpeeds = testChassisSpeeds.unaryMinus();

    assertEquals(-1, invertedSpeeds.vxMetersPerSecond);
    assertEquals(-1, invertedSpeeds.vyMetersPerSecond);
    //    assertEquals(Math.PI, invertedSpeeds.omegaRadiansPerSecond);
  }

  @AfterEach
  void shutdown() throws Exception {}
}
