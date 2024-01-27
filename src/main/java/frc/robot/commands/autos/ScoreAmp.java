// package frc.robot.commands.autos;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.amp.AutoAmpFlipperSetpoints;
// import frc.robot.commands.amp.AutoSetAmpSpeed;
// import frc.robot.constants.AMP.AMP_STATE;
// import frc.robot.constants.AMP.FLIPPER_SETPOINT;
// import frc.robot.constants.FLYWHEEL.WAIT;
// import frc.robot.subsystems.AmpFlipper;
// import frc.robot.subsystems.AmpShooter;

// public class ScoreAmp extends SequentialCommandGroup {

//   public ScoreAmp(AmpFlipper flipper, AmpShooter AmpShooter) {

//     addCommands(
//         new AutoAmpFlipperSetpoints(flipper, FLIPPER_SETPOINT.FORWARD),
//         new AutoSetAmpSpeed(AmpShooter, AMP_STATE.SCORE),
//         new WaitCommand(WAIT.WAIT_FOR_AMP_SCORE.get()),
//         new AutoAmpFlipperSetpoints(flipper, FLIPPER_SETPOINT.STOWED),
//         new AutoSetAmpSpeed(AmpShooter, AMP_STATE.NONE));
//   }
// }
