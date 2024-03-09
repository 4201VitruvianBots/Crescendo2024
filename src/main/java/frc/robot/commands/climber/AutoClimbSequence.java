package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ampShooter.AutoSetAmpSpeed;
import frc.robot.commands.arm.AutoArmSetpoints;
import frc.robot.commands.drive.DriveEndgame;
import frc.robot.constants.AMPSHOOTER;
import frc.robot.constants.ARM;
import frc.robot.constants.CLIMBER.CLIMBER_SETPOINT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoClimbSequence extends SequentialCommandGroup {

  public AutoClimbSequence(
      AmpShooter ampshooter, CommandSwerveDrivetrain swerveDrive, Arm arm, Climber climber) {

    addCommands(

        // will drive forward and set up the arm forward
        // to make room for climber to climb and extend the climber
        new ParallelCommandGroup(
            new DriveEndgame(swerveDrive).withTimeout(1),
            new AutoArmSetpoints(arm, ARM.ARM_SETPOINT.STAGED).withTimeout(1),
            new SetClimberSetpoint(climber, CLIMBER_SETPOINT.EXTEND.getSetpointMeters())).withTimeout(5),
        // will climb then arm will go forward into the trap
        new SetClimberSetpoint(climber, CLIMBER_SETPOINT.FULL_RETRACT.getSetpointMeters()),
        new WaitCommand(2),
        new AutoArmSetpoints(arm, ARM.ARM_SETPOINT.FORWARD),
        new WaitCommand(2),
        new AutoSetAmpSpeed(ampshooter, AMPSHOOTER.STATE.REVERSE)
    );
  }
}
