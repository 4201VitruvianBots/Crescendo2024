package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.AutoSetPercentSetpoint;
import frc.robot.constants.FLYWHEEL.FLYWHEEL_STATE;
import frc.robot.constants.FLYWHEEL.WAIT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Shooter;

public class ScoreSpeaker extends SequentialCommandGroup {

  public ScoreSpeaker(Shooter shooter, AmpShooter uptake) {

    addCommands(
        new AutoSetPercentSetpoint(shooter, FLYWHEEL_STATE.SPEAKER),
        new WaitCommand(WAIT.WAIT_FOR_FLYWHEEL_SETPOINT.get())
        // TODO: Replace with Amp Rollers
        // new AutoRunUptake(uptake, UPTAKE_STATE.DOWNTAKE)
        );
  }
}
