// package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.amp.AutoArmSetpoints;
// import frc.robot.commands.shooter.AutoSetRPMSetpoint;
// import frc.robot.commands.uptake.AutoRunUptake;
// import frc.robot.constants.AMP;
// import frc.robot.constants.CLIMBER;
// import frc.robot.constants.CLIMBER.CLIMBER_SETPOINT;
// import frc.robot.constants.FLYWHEEL.FLYWHEEL_STATE;
// import frc.robot.constants.FLYWHEEL.WAIT;
// import frc.robot.constants.UPTAKE.UPTAKE_STATE;
// import frc.robot.subsystems.AmpShooter;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Uptake;

// public class ClimbFinal extends SequentialCommandGroup {

//   public ClimbFinal(AmpShooter ampshooter, Arm arm, Climber climber) {

//     addCommands(
//       //will set up the arm to about level with the floor and extend the climber
//       new ParallelCommandGroup(
//         new AutoArmSetpoints(arm, AMP.FLIPPER_SETPOINT.STOWED),
//         new SetClimberSetpoint(climber, CLIMBER_SETPOINT.EXTEND)
//           .withTimeout(1)),
//       //will climb then arm will go forward into the trap
//       new WaitCommand(1),
//       new SetClimberSetpoint(climber, CLIMBER_SETPOINT.FULL_RETRACT),
//       new AutoArmSetpoints(arm, AMP.FLIPPER_SETPOINT.FORWARD)
//     );
//   }
// }
