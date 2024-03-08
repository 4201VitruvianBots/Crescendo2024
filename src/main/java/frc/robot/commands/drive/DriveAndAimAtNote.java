// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SWERVE;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class DriveAndAimAtNote extends Command {
  private final CommandSwerveDrivetrain m_SwerveDrivetrain;
  private final Vision m_vision;
  private final DoubleSupplier m_throttleInput;
  private final DoubleSupplier m_strafeInput;
  private final DoubleSupplier m_turnInput;
  private final PIDController m_PidController =
      new PIDController(SWERVE.DRIVE.kAutoP_Theta, SWERVE.DRIVE.kAutoI_Theta, SWERVE.DRIVE.kAutoD_Theta);
  double finalTurn = 0.0;

  /** Creates a new rotateRobotToGoal. */
  public DriveAndAimAtNote(
      CommandSwerveDrivetrain commandSwerveDrivetrain,
      Vision vision,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier turnInput) {
    m_SwerveDrivetrain = commandSwerveDrivetrain;
    m_vision = vision;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_turnInput = turnInput;
    m_PidController.setTolerance(Units.degreesToRadians(2));
    m_PidController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_SwerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.hasGamePieceTarget()) {
      var setPoint =
          m_SwerveDrivetrain
              .getState()
              .Pose
              .getRotation()
              .plus(m_vision.getRobotToGamePieceRotation());
      var turnRate =
          m_PidController.calculate(
              m_SwerveDrivetrain.getState().Pose.getRotation().getRadians(), setPoint.getRadians());
      finalTurn =
          MathUtil.clamp(
              turnRate,
              -SWERVE.DRIVE.kMaxRotationRadiansPerSecond,
              SWERVE.DRIVE.kMaxRotationRadiansPerSecond);
      m_SwerveDrivetrain.setChassisSpeedControl(
          new ChassisSpeeds(
              m_throttleInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
              m_turnInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
              finalTurn));
    } else {
      m_SwerveDrivetrain.setChassisSpeedControl(
          new ChassisSpeeds(
              m_throttleInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
              m_strafeInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
              m_turnInput.getAsDouble() * DRIVE.kMaxRotationRadiansPerSecond));
    }
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
