package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.amp.AutoArmSetpoints;
import frc.robot.commands.amp.AutoSetAmpSpeed;
import frc.robot.commands.intake.AutoRunIntake;
import frc.robot.constants.ARM.AMP_STATE;
import frc.robot.constants.ARM.ARM_SETPOINT;
import frc.robot.constants.INTAKE.INTAKE_STATE;
import frc.robot.constants.SHOOTER.WAIT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ScoreAmp extends SequentialCommandGroup {

  public ScoreAmp(Arm flipper, AmpShooter AmpShooter, Intake intake, INTAKE_STATE state) {

    addCommands(
        new AutoRunIntake(intake, INTAKE_STATE.INTAKING),
        new AutoArmSetpoints(flipper, ARM_SETPOINT.FORWARD),
        new AutoSetAmpSpeed(AmpShooter, AMP_STATE.SCORE),
        new WaitCommand(WAIT.WAIT_FOR_AMP_SCORE.get()),
        new AutoSetAmpSpeed(AmpShooter, AMP_STATE.NONE),
        new AutoArmSetpoints(flipper, ARM_SETPOINT.STOWED));
  }
}
