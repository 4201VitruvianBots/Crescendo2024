package frc.robot.commands.climber;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.amp.AutoArmSetpoints;
import frc.robot.commands.shooter.AutoSetRPMSetpoint;
import frc.robot.constants.ARM;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.ARM.AMP_STATE;
import frc.robot.constants.CLIMBER.CLIMBER_SETPOINT;
import frc.robot.constants.FLYWHEEL.FLYWHEEL_STATE;
import frc.robot.constants.FLYWHEEL.WAIT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class ClimbFinal extends SequentialCommandGroup {

  public ClimbFinal(AmpShooter ampshooter, CommandSwerveDrivetrain swerveDrive, Arm arm, Climber climber) {

    addCommands(

      //will drive forward and set up the arm to about level with the floor and extend the climber
      new ParallelCommandGroup(
        new DriveEndgame(swerveDrive).withTimeout(2),
        new AutoArmSetpoints(arm, ARM.ARM_SETPOINT.STOWED),
        new SetClimberSetpoint(climber, CLIMBER_SETPOINT.EXTEND),

      //will climb then arm will go forward into the trap
      new WaitCommand(1),
      new SetClimberSetpoint(climber, CLIMBER_SETPOINT.FULL_RETRACT),
      new WaitCommand(2),
      new AutoArmSetpoints(arm, ARM.ARM_SETPOINT.FORWARD),
      new WaitCommand(2)
    //   new AutoSetAmpSpeed(ampshooter, AMP_STATE.SCORE)
    ));
  }
}
