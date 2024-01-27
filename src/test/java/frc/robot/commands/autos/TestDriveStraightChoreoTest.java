package frc.robot.commands.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.RobotTime;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestDriveStraightChoreoTest {
  static final double DELTA = 0.02; // acceptable deviation range

  RobotTime m_robotTime;
  SwerveDrive m_swerveDrive;
  FieldSim m_fieldSim;
  Vision m_vision; 

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    m_robotTime = new RobotTime();
    RobotTime.setTimeMode(RobotTime.TIME_MODE.UNITTEST);
    m_swerveDrive = new SwerveDrive(m_vision);
    m_fieldSim = new FieldSim(m_swerveDrive);

    /* enable the robot */
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    /* delay ~100ms so the devices can start up and enable */
    Timer.delay(0.100);
  }

  @Test
  public void testPathLoading() {
    PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("DriveStraightTest");

    var startPose = new Pose2d(2, 2, new Rotation2d());
    var maxVel = 1.0;

    //        assertEquals(path.getGlobalConstraints().getMaxVelocityMps(), maxVel, DELTA);
    assertEquals(startPose, path.getPreviewStartingHolonomicPose());
  }

  @Test
  public void testTestDriveStraightChoreoTestCommand() {
    var command = new DriveStraightChoreoTest(m_swerveDrive, m_fieldSim);

    CommandScheduler.getInstance().enable();
    CommandScheduler.getInstance().registerSubsystem(m_swerveDrive);
    CommandScheduler.getInstance().registerSubsystem(m_fieldSim);
    CommandScheduler.getInstance().schedule(command);
    for (int i = 0; i < 20; i++) {
      CommandScheduler.getInstance().run();
      var cmd = m_swerveDrive.getCurrentCommand();

      System.out.println("TEST");
    }
  }
}
