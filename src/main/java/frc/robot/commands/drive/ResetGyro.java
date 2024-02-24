// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;

public class ResetGyro extends Command {
  /** Creates a new RestGyro. */
  private final CommandSwerveDrivetrain m_swerveDrive;

  public ResetGyro(CommandSwerveDrivetrain swerveDrive) {
    m_swerveDrive = swerveDrive;

    addRequirements(m_swerveDrive);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Controls.isRedAlliance()) {
      m_swerveDrive.resetGyro(0);
    } else {
      m_swerveDrive.resetGyro(180);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
