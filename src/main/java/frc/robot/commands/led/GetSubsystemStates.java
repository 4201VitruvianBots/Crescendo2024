// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class GetSubsystemStates extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDSubsystem m_led;

  private final Climbing m_climber;
  private final Intake m_intake;
  private final StateHandler m_stateHandler;
  private final ;
  private boolean isIntaking;

  /** Sets the LED based on the subsystems' statuses */
  public GetSubsystemStates(
      LEDSubsystem led, Intake intake, StateHandler stateHandler) {
    m_led = led;
    m_stateHandler = stateHandler;
    m_intake = intake;
    
    m_climber = climber;
    addRequirements(m_led);
  }

  public GetSubsystemStates() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isIntaking = m_intake.getIntakeState() == INTAKE_STATE.INTAKING;
    isClimbing = m_climber.getClimberState() == CLIMBER_STATE.CLIMBING;
    isScoringSpeaker = m_shooter.getShooterState() == SHOOTER_STATE.SCORE_SPEAKER;
    
    // the prioritized state to be expressed to the LEDs
    // set in order of priority to be expressed from the least priority to the
    // highest priority
    if (DriverStation.isDisabled()) {
      // I'll figure this out later
    } else {
      switch (m_stateHandler.getDesiredState()) {
          // TODO: Add states for substation intaking
        case INTAKING:
          if (isIntaking) {
            m_led.expressState(SUPERSTRUCTURE_STATE.INTAKING);
          } 
          { else {
                
          }
          break;
        case CLIMBING:
          if (isClimbing) {
            M_led.expressState(SUPERSTRUCTURE_STATE.CLIMBING);
          } else {
            m_led.expressState(SUPERSTRUCTURE_STATE.FINISHEDCLIMBING); 
            
          }
          break;
        case SCORE_SPEAKER:
          if (isScoringSpeaker) {
            m_led.expressState(SUPERSTRUCTURE_STATE.SCORE_SPEAKER);
            
          }
          break;
        case SCORE_TRAP:
        case SCORE_AMP:
          if (isScoringAmp) {
            m_led.expressState(SUPERSTRUCTURE_STATE.SCORE_AMP);
          } else {
            m_led.expressState(SUPERSTRUCTURE_STATE.SCORE_TRAP);
          }
          break;
        default:
          m_led.expressState(SUPERSTRUCTURE_STATE.ENABLED);
          break;
      }
    }
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