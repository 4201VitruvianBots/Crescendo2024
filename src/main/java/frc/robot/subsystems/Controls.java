package frc.robot.subsystems;

import static frc.robot.constants.AUTO.AUTO_POSE_MAP;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ARM;
import frc.robot.constants.FIELD;
import frc.robot.constants.ROBOT;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("RedundantThrows")
public class Controls extends SubsystemBase implements AutoCloseable {
  private CommandSwerveDrivetrain m_swerveDrive;
  private Intake m_intake;
  private Arm m_arm;
  private Vision m_vision;
  private Pose2d m_startPose = new Pose2d(-1, -1, new Rotation2d());
  private static DriverStation.Alliance m_allianceColor = DriverStation.Alliance.Red;

  private static boolean m_initState;
  private final Alert m_initStateAlert =
      new Alert("RobotInit", "Robot is not ready to start a match!", Alert.AlertType.ERROR);
  private final Alert m_initPoseAlert =
      new Alert("RobotInit", "Robot Pose is not in the correct location!", Alert.AlertType.ERROR);
  private final Alert m_initArmAlert =
      new Alert("RobotInit", "Robot Arm is not initialized!", Alert.AlertType.ERROR);

  public Controls() {
    m_initState = false;

    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get())
      Logger.recordOutput("Controls/Robot Serial Number", RobotController.getSerialNumber());
  }

  public void registerDriveTrain(CommandSwerveDrivetrain swerveDrive) {
    m_swerveDrive = swerveDrive;
  }

  public void registerIntake(Intake intake) {
    m_intake = intake;
  }

  public void registerArm(Arm arm) {
    m_arm = arm;
  }

  public void registerVision(Vision vision) {
    m_vision = vision;
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

  public static boolean isBlueAlliance() {
    return (m_allianceColor == DriverStation.Alliance.Blue);
  }

  public void setPDHChannel(boolean on) {
    // pdh.setSwitchableChannel(on);
  }

  public static boolean getInitState() {
    return m_initState;
  }

  public void resetInitState() {
    m_initState = false;

    if (m_vision != null) {
      m_vision.resetInitialLocalization();
    }
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
      m_initState = true;

      // Check if the robot is set in the correct position for the selected auto
      if (m_swerveDrive != null) {
        var poseDiff = m_startPose.minus(m_swerveDrive.getState().Pose);
        if (Math.abs(poseDiff.getX()) > 0.1
            || Math.abs(poseDiff.getY()) > 0.1
            || Math.abs(poseDiff.getRotation().getDegrees()) > 5.0) {
          m_initState = false;
          m_initPoseAlert.setText("Robot Pose is not in the correct location!");
          m_initPoseAlert.set(true);
        } else {
          m_initPoseAlert.setText("Robot Pose is in the correct location!");
          m_initPoseAlert.set(false);
        }
      }

      // Check if the intake detects a note
      if (m_intake != null) {
        m_initState = !m_intake.checkEitherIntakeSensorActive();
      }

      // Check if the robot arm was initialized
      if (m_arm != null) {
        // TODO: Check if this is valid
        if (m_arm.getCurrentAngle() == ARM.startingAngleDegrees) {
          m_initState = false;
          m_initArmAlert.setText("Robot Arm is not initialized!");
          m_initArmAlert.set(true);
        } else {
          m_initArmAlert.setText("Robot Arm is initialized!");
          m_initArmAlert.set(false);
        }
      }

      if (!m_initState) {
        m_initStateAlert.setText("Robot is not ready to start a match!");
        m_initStateAlert.set(true);
      } else {
        m_initStateAlert.setText("Robot is ready to start a match!");
        m_initStateAlert.set(false);
      }
    }
  }

  public void updateStartPose(String autoName) {
    if (autoName != null && AUTO_POSE_MAP.containsKey(autoName)) {
      m_startPose = FIELD.pathPlannerFlip(AUTO_POSE_MAP.get(autoName).get());
    }

    if (m_swerveDrive != null) {
      var deltaTranslation =
          m_startPose.getTranslation().minus(m_swerveDrive.getState().Pose.getTranslation());
      var deltaRotation =
          m_startPose.getRotation().minus(m_swerveDrive.getState().Pose.getRotation());

      Logger.recordOutput("Controls/startDeltaTranslation", deltaTranslation);
      SmartDashboard.putNumber("Controls/robotPoseX", m_swerveDrive.getState().Pose.getX());
      SmartDashboard.putNumber("Controls/robotPoseY", m_swerveDrive.getState().Pose.getY());
      SmartDashboard.putNumber(
          "Controls/robotPoseDegrees", m_swerveDrive.getState().Pose.getRotation().getDegrees());
      SmartDashboard.putNumber("Controls/startPoseX", m_startPose.getX());
      SmartDashboard.putNumber("Controls/startPoseY", m_startPose.getY());
      SmartDashboard.putNumber("Controls/startPoseDegrees", m_startPose.getRotation().getDegrees());
      SmartDashboard.putNumber("Controls/startDeltaTranslationX", deltaTranslation.getX());
      SmartDashboard.putNumber("Controls/startDeltaTranslationY", deltaTranslation.getY());
      SmartDashboard.putNumber("Controls/startDeltaRotation", deltaRotation.getDegrees());
    }
  }

  /** Sends values to SmartDashboard */
  private void updateLogger() {
    try {
      Logger.recordOutput("Controls/AllianceColor", getAllianceColor());
      Logger.recordOutput("Controls/StartPose", getStartPose());
    } catch (Exception e) {
      System.out.println("Controls failed to update AdvantageKit");
    }
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation() || (RobotBase.isReal() && DriverStation.isDisabled())) {
      updateAllianceColor();
    }
    updateInitState();

    // This method will be called once per scheduler run
    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get()) updateLogger();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {}
}
