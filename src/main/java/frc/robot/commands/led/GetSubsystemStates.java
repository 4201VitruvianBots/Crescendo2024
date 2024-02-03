// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LED;
import frc.robot.subsystems.*;

public class GetSubsystemStates extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDSubsystem m_led;

  private final AmpShooter m_ampShooter;
  private final Climber m_climber;
  private final Intake m_intake;
  private final Shooter m_shooter;
  private boolean isIntaking;
  private boolean isClimbing;
  private boolean isScoringSpeaker;
  private boolean isUptaking;
  private boolean isDisabled;
  private boolean isEnabled;

  /** Sets the LED based on the subsystems' statuses */
  public GetSubsystemStates(
      LEDSubsystem led, Intake intake, Climber climber, AmpShooter ampShooter, Shooter shooter) {
    m_led = led;
    m_intake = intake;
    m_climber = climber;
    m_ampShooter = ampShooter;
    m_shooter = shooter;

    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // isIntaking = m_intake.getIntakeState() == INTAKE_STATE.INTAKING;
    // isUptaking = m_uptake.getUptakeState() == UPTAKE_STATE.UPTAKING;
    isDisabled = DriverStation.isDisabled();
    isEnabled = !isDisabled;
    // isClimbing = m_climber.getClimberState() == SUBSYSTEM_STATES.CLIMBING;
    // isScoringSpeaker = m_shooter.getShooterState() == SUBSYSTEM_STATES.SCORE_SPEAKER;

    // the prioritized state to be expressed to the LEDs
    // set in order of priority to be expressed from the least priority to the
    // highest priority
    if (isDisabled) {
      m_led.expressState(LED.SUBSYSTEM_STATES.DISABLED);
    } else if (isIntaking) {
      m_led.expressState(LED.SUBSYSTEM_STATES.INTAKING);
    } else if (isUptaking) {
      m_led.expressState(LED.SUBSYSTEM_STATES.UPTAKING);
    } else if (isEnabled) {
      m_led.expressState(LED.SUBSYSTEM_STATES.ENABLED);
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
