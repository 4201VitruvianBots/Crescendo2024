// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

<<<<<<<< HEAD:src/main/java/frc/robot/commands/shooter/SetAndHoldPercentSetpoint.java
public class SetAndHoldPercentSetpoint extends Command {
  Shooter m_shooter;
  double m_percentOutput;

  public SetAndHoldPercentSetpoint(Shooter flywheel, double percentOutput) {
    m_shooter = flywheel;
========
public class SetAndHoldPercentOutputSetpoint extends Command {
  Shooter m_shooter;
  double m_percentOutput;

  public SetAndHoldPercentOutputSetpoint(Shooter shooter, double percentOutput) {
    m_shooter = shooter;
>>>>>>>> origin/47-change-rpm-to-percent-output:src/main/java/frc/robot/commands/shooter/SetAndHoldPercentOutputSetpoint.java
    m_percentOutput = percentOutput;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.setPercentOutput(m_percentOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
