// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LED;
import frc.robot.constants.VISION.TRACKING_STATE;
import frc.robot.subsystems.*;

public class GetSubsystemStates extends Command {
  private final LEDSubsystem m_led;
  private final Climber m_climber;
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final Vision m_vision;
  private final CommandSwerveDrivetrain m_SwerveDrivetrain;
  // private final Arm m_arm;

  private boolean isClimbing;
  private boolean isUnreved;
  private boolean isReved;
  private boolean isIntaking;
  private boolean isEnabled;
  private boolean isSetup;
  private boolean isLocalized;
  private boolean isDisabled;
  private boolean isIntaked;
  private boolean isLocked;

  /** Sets the LED based on the subsystems' statuses */
  public GetSubsystemStates(
      LEDSubsystem led, Intake intake, Climber climber, Shooter shooter, Vision vision, CommandSwerveDrivetrain swerveDrive) {
    m_led = led;
    m_intake = intake;
    m_climber = climber;
    // m_arm = arm;
    m_SwerveDrivetrain = swerveDrive;
    m_shooter = shooter;
    m_vision = vision;

    addRequirements(m_led);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isClimbing = m_climber.getClimbState();

   isLocked = m_SwerveDrivetrain.isTrackingState();
    
    isReved = m_shooter.getReved();
    isUnreved = m_shooter.getUnreved(); // Done
    isIntaked = m_intake.checkEitherIntakeSensorActive();
    isIntaking = m_intake.getIntakeState(); // Done
    isEnabled = DriverStation.isEnabled();
    isSetup = Controls.getInitState();
    isLocalized = m_vision.getInitialLocalization();
    isDisabled = DriverStation.isDisabled(); // Done
    // after it's done.
    // isArmScoring = m_arm.getArmState(); // Done

    // the prioritized state to be expressed to the LEDs
    // set in order of priority to be expressed from the least priority to the
    // highest priority
    if (isClimbing) {
      m_led.expressState(LED.SUBSYSTEM_STATES.CLIMBING);
    } else if (isLocked) {
      m_led.expressState(LED.SUBSYSTEM_STATES.LOCKED);
    } else if (isUnreved) {
      m_led.expressState(LED.SUBSYSTEM_STATES.UNREVED);
    } else if (isIntaked) {
      m_led.expressState(LED.SUBSYSTEM_STATES.INTAKED);
    } else if (isIntaking) {
      m_led.expressState(LED.SUBSYSTEM_STATES.INTAKING);
    } else if (isEnabled) {
      m_led.expressState(LED.SUBSYSTEM_STATES.ENABLED);
    } else if (isSetup) {
      m_led.expressState(LED.SUBSYSTEM_STATES.SETUP_READY);
    } else if (isLocalized) {
      m_led.expressState(LED.SUBSYSTEM_STATES.SETUP_LOCALIZED);
    } else if (isDisabled) {
      m_led.expressState(LED.SUBSYSTEM_STATES.DISABLED);
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
