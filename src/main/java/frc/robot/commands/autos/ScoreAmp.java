package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ampShooter.AutoSetAmpSpeed;
import frc.robot.commands.arm.AutoArmSetpoints;
import frc.robot.commands.intake.AutoRunAmpTake;
import frc.robot.constants.AMPSHOOTER;
import frc.robot.constants.ARM.ARM_SETPOINT;
import frc.robot.constants.INTAKE;
import frc.robot.constants.SHOOTER.WAIT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ScoreAmp extends SequentialCommandGroup {

  public ScoreAmp(Arm flipper, AmpShooter AmpShooter, Intake intake) {

    addCommands(
        new AutoRunAmpTake(
            intake,
            AmpShooter,
            INTAKE.STATE.FRONT_ROLLER_INTAKING.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
            AMPSHOOTER.STATE.INTAKING.get()),
        new AutoArmSetpoints(flipper, ARM_SETPOINT.FORWARD),
        new AutoSetAmpSpeed(AmpShooter, AMPSHOOTER.STATE.INTAKING),
        new WaitCommand(WAIT.WAIT_FOR_AMP_SCORE.get()),
        new AutoSetAmpSpeed(AmpShooter, AMPSHOOTER.STATE.NONE),
        new AutoArmSetpoints(flipper, ARM_SETPOINT.STOWED));
  }
}
