// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.ResourceBundle.Control;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Controls;

// public class RestOdometry extends Command {
//   /** Creates a new RestGyro. */

//   private static DriverStation.Alliance m_allianceColor = DriverStation.Alliance.Red;
//    CommandSwerveDrivetrain m_swervedrive;
//   public RestOdometry(CommandSwerveDrivetrain swervedrive) {
// m_swervedrive = swervedrive;

//     addRequirements(m_swervedrive);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     if ( m_allianceColor == DriverStation.Alliance.Red ){
//       m_swervedrive.resetGyro(180);
//     }
//     else if(m_allianceColor == DriverStation.Alliance.Blue ){
//       m_swervedrive.resetGyro(0);
//     }
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }
