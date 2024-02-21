// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// // Called when the joystick moves up/down, also acts as manual override
// package frc.robot.commands.climber;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.constants.ROBOT.CONTROL_MODE;
// import frc.robot.subsystems.Climber;

// public class AutoSetClimberSetpoint extends Command {
//   /** Creates a new IncrementElevatorHeight. */
//   private final Climber m_climber;

//   private final double m_setpoint;

//   public AutoSetClimberSetpoint(Climber climber, double setpoint) {
//     m_climber = climber;
//     m_setpoint = setpoint;

//     addRequirements(m_climber);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_climber.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_climber.setDesiredPositionMeters(m_setpoint);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   // 1 inch = 0.254 meters
//   public boolean isFinished() {
//     return (Math.abs(m_climber.getHeightMeters() - m_setpoint) < Units.inchesToMeters(1));
//   }
// }
