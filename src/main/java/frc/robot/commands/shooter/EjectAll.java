// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;

public class SetAndHoldRPMSetpoint extends Command {
  /** Creates a new ResetGyro. */
  private final Shooter m_shooter;
  private double m_RPM;


  public SetAndHoldRPMSetpoint() {
    m_shooter = shooter;
    m_RPM1 = RPM1;
    m_RPM2 = RPM2;

    addRequirements(m_shooter);
  }
  
 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(Shooter shooter, double RPM) {


    m_shooter.setRPM1(RPM);
    m_shooter.setRPM2(RPM*0.9);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
