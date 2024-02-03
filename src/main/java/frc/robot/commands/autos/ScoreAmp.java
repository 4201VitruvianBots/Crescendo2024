package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.amp.AutoArmSetpoints;
import frc.robot.commands.amp.AutoSetAmpSpeed;
import frc.robot.constants.AMP.AMP_STATE;
import frc.robot.constants.AMP.FLIPPER_SETPOINT;
import frc.robot.constants.FLYWHEEL.WAIT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Arm;

public class ScoreAmp extends SequentialCommandGroup {

  public ScoreAmp(Arm flipper, AmpShooter AmpShooter) {

    addCommands(
        new AutoArmSetpoints(flipper, FLIPPER_SETPOINT.FORWARD),
        new AutoSetAmpSpeed(AmpShooter, AMP_STATE.SCORE),
        new WaitCommand(WAIT.WAIT_FOR_AMP_SCORE.get()),
        new AutoArmSetpoints(flipper, FLIPPER_SETPOINT.STOWED),
        new AutoSetAmpSpeed(AmpShooter, AMP_STATE.NONE));
  }
}
