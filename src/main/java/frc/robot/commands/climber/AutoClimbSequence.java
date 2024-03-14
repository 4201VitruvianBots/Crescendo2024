package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ampShooter.AutoSetAmpSpeed;
import frc.robot.commands.arm.AutoArmSetpoints;
import frc.robot.commands.arm.SetArmMaxSpeed;
import frc.robot.constants.AMPSHOOTER;
import frc.robot.constants.ARM;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoClimbSequence extends SequentialCommandGroup {

  public AutoClimbSequence(
      CommandSwerveDrivetrain swerveDrive, Climber climber, Arm arm, AmpShooter ampshooter) {

    addCommands(
        // put arm at set point and hold (same position as amp)
        new AutoArmSetpoints(arm, ARM.ARM_SETPOINT.TRAP),
        // adjust note up into trap position (7 motor rotations in outtake direction at 30% speed)
        new AutoSetAmpSpeed(ampshooter, AMPSHOOTER.STATE.OUTTAKE).withTimeout(0.15),
        // put climbers down at 50% speed until both are at/around 0 position (bottom), then hold climber
        new AutoRetractClimber(climber),
        // spit note down (shooting direction) for 0.5 seconds
        new AutoSetAmpSpeed(ampshooter, AMPSHOOTER.STATE.SHOOTING).withTimeout(0.5),
        // then slowly lower arm about 15 degrees at 20% speed
        new SetArmMaxSpeed(arm, 0.2),
        new AutoArmSetpoints(arm, ARM.ARM_SETPOINT.TRAP_LOWER),
        // sequence stop, leave everything in brake mode
        new EndgameBrake(climber, arm, ampshooter)
    );
  }
}
