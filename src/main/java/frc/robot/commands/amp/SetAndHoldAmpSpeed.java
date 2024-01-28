// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.amp;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.AmpShooter;

// public class SetAndHoldAmpSpeed extends Command {
//   AmpShooter m_ampShooter;
//   double m_speed;

//   public SetAndHoldAmpSpeed(AmpShooter ampShooter, double RPM) {
//     m_ampShooter = ampShooter;
//     addRequirements(m_ampShooter);

//     m_speed = RPM;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   @Override
//   public void execute() {
//     m_ampShooter.setPercentOutput(m_speed);
//   }

//   // Called every time the scheduler runs while the command is scheduled.

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }
