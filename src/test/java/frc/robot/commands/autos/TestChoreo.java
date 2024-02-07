package frc.robot.commands.autos;

import static frc.robot.constants.SWERVE.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.RobotTime;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Telemetry;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

@Disabled("To Fix after Swerve Code Updates")
public class TestChoreo {
  static final double DELTA = 0.02; // acceptable deviation range

  RobotTime m_robotTime;
  CommandSwerveDrivetrain m_swerveDrive;
  Telemetry m_telemetry;
  FieldSim m_fieldSim;
  Shooter m_shooter;

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    m_robotTime = new RobotTime();
    RobotTime.setTimeMode(RobotTime.TIME_MODE.UNITTEST);
    m_swerveDrive =
        new CommandSwerveDrivetrain(
            DrivetrainConstants,
            FrontLeftConstants,
            FrontRightConstants,
            BackLeftConstants,
            BackRightConstants);
    m_telemetry = new Telemetry();
    m_shooter = new Shooter();
    m_fieldSim = new FieldSim(m_shooter);
    m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
    m_telemetry.registerFieldSim(m_fieldSim);

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
  public void testTestDriveStraightCommand() {
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
