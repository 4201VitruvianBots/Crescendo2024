package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.AutoRunAll;
import frc.robot.commands.intake.AutoRunAmpTake;
import frc.robot.constants.*;
import frc.robot.constants.INTAKE.STATE;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
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
                AMPSHOOTER.STATE.INTAKING.get())
            .withTimeout(2),
        new WaitCommand(2),
        // new AutoSetAmpSpeed(ampShooter, AMP.STATE.INTAKING).withTimeout(1),
        // new WaitCommand(1),
        new AutoRunAll(
            intake,
            shooter,
            ampShooter,
            STATE.FRONT_ROLLER_INTAKING.get(),
            STATE.BACK_ROLLER_INTAKING.get(),
            AMPSHOOTER.STATE.INTAKING.get(),
            RPM_SETPOINT.MAX.get()));
  }
}
