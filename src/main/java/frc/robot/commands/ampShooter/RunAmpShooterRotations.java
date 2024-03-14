// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ampShooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpShooter;

public class RunAmpShooterRotations extends Command {
  private final AmpShooter m_ampShooter;
  
  /** Creates a new RunAmpShooterRotations. */
  public RunAmpShooterRotations(AmpShooter ampShooter) {
    m_ampShooter = ampShooter;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ampShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
