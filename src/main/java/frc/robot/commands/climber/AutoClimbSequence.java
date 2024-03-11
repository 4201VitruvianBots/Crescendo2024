package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.AutoArmSetpoints;
import frc.robot.constants.ARM;
import frc.robot.constants.CLIMBER.CLIMBER_SETPOINT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoClimbSequence extends SequentialCommandGroup {

  public AutoClimbSequence(
          CommandSwerveDrivetrain swerveDrive, Climber climber, Arm arm, AmpShooter ampshooter) {

    addCommands(

        // will drive forward and set up the arm forward
        // to make room for climber to climb and extend the climber
        new ParallelCommandGroup(
            // new DriveEndgame(swerveDrive).withTimeout(2),
            new AutoArmSetpoints(arm, ARM.ARM_SETPOINT.FORWARD),
            new WaitCommand(1),
            new SetClimberSetpoint(climber, CLIMBER_SETPOINT.EXTEND.getSetpointMeters())));

    // will climb then arm will go forward into the trap
    // new WaitCommand(1),
    // new AutoSetSetpoint(climber, CLIMBER_SETPOINT.FULL_RETRACT),
    // new WaitCommand(2),
    // new AutoArmSetpoints(arm, ARM.ARM_SETPOINT.STOWED),
    // new WaitCommand(2),
    // new AutoSetAmpSpeed(ampshooter, AMP.STATE.REVERSE));
  }
}
