package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FIELD;
import frc.robot.constants.SWERVE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import java.util.function.DoubleSupplier;

public class DriveAndAim extends Command {
  private final CommandSwerveDrivetrain m_swerveDrive;
  private final DoubleSupplier m_throttleInput;
  private final DoubleSupplier m_strafeInput;
  private final PIDController m_turnController =
      new PIDController(SWERVE.DRIVE.kP_Theta, SWERVE.DRIVE.kI_Theta, SWERVE.DRIVE.kD_Theta);
  private Translation2d m_target = new Translation2d();

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDrive The subsystem used by this command.
   */
  public DriveAndAim(
      CommandSwerveDrivetrain swerveDrive,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput) {
    m_swerveDrive = swerveDrive;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;

    m_turnController.enableContinuousInput(-Math.PI, Math.PI);
    m_turnController.setTolerance(Units.degreesToRadians(2.0));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Controls.isRedAlliance()) {
      m_target = FIELD.redSpeaker;
    } else {
      m_target = FIELD.blueSpeaker;
    }
    var targetDelta = m_swerveDrive.getState().Pose.getTranslation().minus(m_target).getAngle();

    m_swerveDrive.setChassisSpeedControl(
        new ChassisSpeeds(
            m_throttleInput.getAsDouble(),
            m_strafeInput.getAsDouble(),
            m_turnController.calculate(
                m_swerveDrive.getState().Pose.getRotation().getRadians(),
                targetDelta.getRadians())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
