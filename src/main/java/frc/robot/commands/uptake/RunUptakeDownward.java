// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.uptake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Uptake;

public class RunUptakeDownward extends Command {
  Uptake m_uptake;
  
  /** Creates a new RunIntake. */
  public RunUptakeDownward(Uptake uptake) {
    m_uptake = uptake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_uptake.setSpeed(-0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_uptake.setSpeed(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_uptake.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

