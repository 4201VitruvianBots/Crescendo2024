// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.shooter;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.constants.FLYWHEEL.FLYWHEEL_STATE;
// import frc.robot.constants.SWERVE.DRIVE;
// import frc.robot.RobotContainer;
// import frc.robot.constants.SWERVE;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.AmpShooter;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Vision;
// import java.util.function.DoubleSupplier;



// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

// public class ShootNStrafe extends Command {
//   Shooter m_shooter;
//   AmpShooter m_ampShooter;
//   FLYWHEEL_STATE m_state;
//   CommandSwerveDrivetrain m_swerveDrive;
//   private double m_percentOutput;


//   private double RPMThreshold = 1200.0;
//   private double timerThreshold = 0.5;

//   private double ChangeThisValue;
  
//   private final Timer m_timer = new Timer();
//   private boolean timerStart = false;

//   private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;
//   private double PoseX = m_swerveDrive.getState().Pose.getX();
//   private double PoseY = m_swerveDrive.getState().Pose.getY();
//   private double shootangle = m_shooter.shootangle(PoseX,PoseY);
  
//   private double displacementX = ChangeThisValue * Math.sin(shootangle); //TODO: Change this value
//   private double displacementY = ChangeThisValue * Math.cos(shootangle);
  
  
//   private double VelocityY =
//     m_swerveDrive.getChassisSpeed().omegaRadiansPerSecond * m_swerveDrive.getState().Pose.getRotation().getSin();
//   private double VelocityX =
//       m_swerveDrive.getChassisSpeed().omegaRadiansPerSecond * m_swerveDrive.getState().Pose.getRotation().getCos();
//   private double VelocityShoot = 1.2; //TODO: Change after testing

//   double m_headingOffset =
//       Math.asin(
//           Math.abs(
//               (displacementY * VelocityX - displacementX * VelocityY)
//                   / ((Math.sqrt(Math.pow(displacementX, 2) + Math.pow(displacementY, 2)))
//                   * VelocityShoot)));

//   private final SwerveRequest.FieldCentric drive =
//       new SwerveRequest.FieldCentric()
//           .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
//           .withRotationalDeadband(
//               SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1) // Add a 10% deadband
//           .withDriveRequestType(
//               SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

//   public ShootNStrafe(
//       CommandSwerveDrivetrain swerveDrive,
//       Vision vision,
//       AmpShooter ampShooter,
//       DoubleSupplier throttleInput,
//       DoubleSupplier strafeInput,
//       DoubleSupplier rotationInput,
//       double percentOutput) {
//     m_swerveDrive = swerveDrive;
//     m_ampShooter = ampShooter;
//     m_throttleInput = throttleInput;
//     m_strafeInput = strafeInput;
//     m_rotationInput = rotationInput;
//     m_percentOutput = percentOutput;
    

//     addRequirements(m_shooter);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timerStart = false;
//   }

//   @Override
//   public void execute() {

//     m_shooter.setRpmOutput(m_percentOutput);

//     double throttle =
//         MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()), 0.05)
//             * Math.signum(m_throttleInput.getAsDouble());
//     double strafe =
//         MathUtil.applyDeadband(Math.abs(m_strafeInput.getAsDouble()), 0.05)
//             * Math.signum(m_strafeInput.getAsDouble());
//     double rotation =
//         MathUtil.applyDeadband(Math.abs(m_rotationInput.getAsDouble()), 0.05)
//             * Math.signum(m_rotationInput.getAsDouble());


//   if (CorrectRange == true && m_shooter.getRPM() > RPMThreshold) { 
      
//       m_shooter.setRpmOutput(RPMThreshold);
//       drive.withVelocityX(VelocityX)
//         .withVelocityY(VelocityY)
//           .withRotationalRate(m_headingOffset);
//             m_ampShooter.setPercentOutput(-0.8);
//             m_timer.reset();
//             m_timer.start();
//             timerStart = true;

//     if(timerStart == true && m_timer.hasElapsed(timerThreshold) ){
//       isFinished();
//     }
//   }

//   else {
//     m_swerveDrive.setControl(
//       drive.withVelocityX((throttle) * DRIVE.kMaxSpeedMetersPerSecond)
//             .withVelocityY((strafe)* DRIVE.kMaxSpeedMetersPerSecond)
//             .withRotationalRate(m_headingOffset));
//       m_timer.reset();
//       m_timer.stop();
//       timerStart = false;
//   }


   
   
//   }

//   // Called every time the scheduler runs while the command is scheduled.

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {

//     m_shooter.setPercentOutput(0);
//     m_ampShooter.setPercentOutput(0);
      
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }
