// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LED;
import frc.robot.constants.LED.SUBSYSTEM_STATES;
import frc.robot.subsystems.*;

public class SetSubsystemStates extends Command {
  private final LEDSubsystem m_led;

  
private final SUBSYSTEM_STATES m_state;

  /** Sets the LED based on the subsystems' statuses */
  public  SetSubsystemStates(
      LEDSubsystem led, SUBSYSTEM_STATES state) {
    m_led = led;
    m_state = state;

    addRequirements(m_led);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {m_led.expressState(m_state);}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
