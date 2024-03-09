// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SWERVE;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveEndgame extends Command {
  private final CommandSwerveDrivetrain m_swerveDrive;

  /** Creates a new LimitClimberJoystickInput. */
  private final SwerveRequest.RobotCentric drive =
      new SwerveRequest.RobotCentric()
          .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
          .withRotationalDeadband(SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  public DriveEndgame(CommandSwerveDrivetrain swerveDrive) {
    m_swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.setControl(drive.withVelocityX(2).withVelocityY(0).withRotationalRate(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
