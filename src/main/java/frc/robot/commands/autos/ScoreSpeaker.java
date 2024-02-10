package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.AutoSetRPMSetpoint;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.constants.SHOOTER.WAIT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Shooter;

public class ScoreSpeaker extends SequentialCommandGroup {

  public ScoreSpeaker(Shooter shooter, AmpShooter uptake) {

    addCommands(
        new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.COOLVALUE.get())
            .withTimeout(WAIT.WAIT_FOR_FLYWHEEL_SETPOINT.get()),
        new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.NONE.get())
        // TODO: Replace with Amp Rollers
        // new AutoRunUptake(uptake, UPTAKE_STATE.DOWNTAKE)
        );
  }
}
