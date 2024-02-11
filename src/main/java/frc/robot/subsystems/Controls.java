package frc.robot.subsystems;

import static frc.robot.constants.AUTO.AUTO_POSE_MAP;

import java.nio.BufferOverflowException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ROBOT;
import frc.robot.simulation.SimConstants;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("RedundantThrows")
public class Controls extends SubsystemBase implements AutoCloseable {
  private CommandSwerveDrivetrain m_swerveDrive;
  private boolean m_initState;
  private Pose2d m_startPose = new Pose2d();
  private static DriverStation.Alliance m_allianceColor = DriverStation.Alliance.Red;

  public Controls() {
    m_initState = false;

    if (!ROBOT.disableLogging)
      Logger.recordOutput("Controls/Robot Serial Number", RobotController.getSerialNumber());
  }

  public void registerDriveTrain(CommandSwerveDrivetrain swerveDrive) {
    m_swerveDrive = swerveDrive;
  }

  /**
   * Returns the robot's current alliance color
   *
   * @return Returns the current alliance color.
   */
  public static DriverStation.Alliance getAllianceColor() {
    return m_allianceColor;
  }

  public static boolean isRedAlliance() {
    return (m_allianceColor == DriverStation.Alliance.Red);
  }

  public static boolean isBlueAllaince() {
    return (m_allianceColor == DriverStation.Alliance.Blue);
  }

  public void setPDHChannel(boolean on) {
    // pdh.setSwitchableChannel(on);
  }

  public boolean getInitState() {
    return m_initState;
  }

  public Pose2d getStartPose() {
    return m_startPose;
  }

  /**
   * Periodically check the DriverStation to get the Alliance color. This mainly runs when the robot
   * is disabled to avoid a bug where the robot tries to get the alliance color before it is
   * connected to a driver station.
   */
  private void updateAllianceColor() {
    var checkDsAlliance = DriverStation.getAlliance();

    checkDsAlliance.ifPresent(alliance -> m_allianceColor = alliance);
  }

  /**
   * TODO: Implement this 1. Check that arm is initialized 2. Check that robot position is close to
   * selected auto position 3.
   */
  private void updateInitState() {
    if (DriverStation.isDisabled()) {
      if (m_swerveDrive != null) {
        var poseDiff = m_startPose.minus(m_swerveDrive.getState().Pose);
        if (Math.abs(poseDiff.getX()) > 0.1
            || Math.abs(poseDiff.getY()) > 0.1
            || Math.abs(poseDiff.getRotation().getDegrees()) > 5.0) {
          m_initState = false;
        }
      }
    }
  }

  public void updateStartPose(String autoName) {
    m_startPose =
        AUTO_POSE_MAP.get(autoName) == null
            ? new Pose2d(-1, -1, new Rotation2d())
            : AUTO_POSE_MAP.get(autoName).get();

    if (isRedAlliance()) {
      m_startPose = SimConstants.allianceFlip(m_startPose);
    }
  }

  /** Sends values to SmartDashboard */
  private void updateLogger() {
    Logger.recordOutput("Controls/AllianceColor", getAllianceColor());
    try {
        Logger.recordOutput("Controls/StartPose", getStartPose());
    } catch (BufferOverflowException e) {
      System.out.println("Failed to record StartPose");
    }
    
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation() || (RobotBase.isReal() && DriverStation.isDisabled())) {
      updateAllianceColor();
    }
    updateInitState();

    // This method will be called once per scheduler run
    if (!ROBOT.disableLogging) updateLogger();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {}
}
