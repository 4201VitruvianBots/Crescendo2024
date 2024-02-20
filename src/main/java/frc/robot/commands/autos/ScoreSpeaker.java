package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.amp.AutoSetAmpSpeed;
import frc.robot.commands.intake.AutoRunAmpTake;
import frc.robot.commands.shooter.AutoScore;
import frc.robot.constants.*;
import frc.robot.constants.SHOOTER.WAIT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ScoreSpeaker extends SequentialCommandGroup {

  public ScoreSpeaker(Shooter shooter, AmpShooter ampShooter, Intake intake) {

    addCommands(
        new AutoRunAmpTake(
                intake,
                ampShooter,
                INTAKE.STATE.FRONT_ROLLER_INTAKING.get(),
                INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
                AMP.STATE.INTAKING.get())
            .withTimeout(2),
        new WaitCommand(1),
        new AutoSetAmpSpeed(ampShooter, AMP.STATE.INTAKING).withTimeout(1),
        new WaitCommand(1),
        new AutoScore(
            shooter,
            ampShooter,
            intake,
            AMP.STATE.INTAKING.get(),
            SHOOTER.RPM_SETPOINT.SPEAKER.get(),
            INTAKE.STATE.FRONT_ROLLER_INTAKING.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
            WAIT.SHOOTING.get(),
            5));
  }
}
