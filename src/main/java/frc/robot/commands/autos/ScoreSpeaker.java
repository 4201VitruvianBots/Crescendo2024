package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.AutoSetPercentSetpoint;
import frc.robot.commands.uptake.AutoRunUptake;
import frc.robot.constants.FLYWHEEL.FLYWHEEL_STATE;
import frc.robot.constants.FLYWHEEL.WAIT;
import frc.robot.constants.UPTAKE.UPTAKE_STATE;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Uptake;

public class ScoreSpeaker extends SequentialCommandGroup {

  public ScoreSpeaker(Flywheel flywheel, Uptake uptake) {

    addCommands(
        new AutoSetPercentSetpoint(flywheel, FLYWHEEL_STATE.SPEAKER),
        new WaitCommand(WAIT.WAIT_FOR_FLYWHEEL_SETPOINT.get()),
        new AutoRunUptake(uptake, UPTAKE_STATE.DOWNTAKE));
  }
}
