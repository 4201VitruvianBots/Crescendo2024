// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class runSwerve extends Command {

  private final CommandSwerveDrivetrain m_swerveDrive;

  private final DoubleSupplier m_throttleInput, m_strafeInput;

  /** Creates a new runSwerve. */
  public runSwerve( CommandSwerveDrivetrain swerveDrive, DoubleSupplier throttleInput,
      DoubleSupplier strafeInput) {
    
        m_swerveDrive = swerveDrive;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    m_swerveDrive.setDefaultCommand(
      m_swerveDrive.applyFieldCentricDrive(
          () ->
              new ChassisSpeeds(
                  m_throttleInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
                  m_strafeInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,0)));
 }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.setDefaultCommand(
      m_swerveDrive.applyFieldCentricDrive(
          () ->
              new ChassisSpeeds(
                  m_throttleInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
                  m_strafeInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,0)));
 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {    m_swerveDrive.setDefaultCommand(
      m_swerveDrive.applyFieldCentricDrive(
          () ->
              new ChassisSpeeds(
                  m_throttleInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
                  m_strafeInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,0)));
 }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
