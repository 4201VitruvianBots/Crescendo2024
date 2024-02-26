// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.climber;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.constants.ROBOT.CONTROL_MODE;
// import frc.robot.subsystems.Climber;
// import java.util.function.DoubleSupplier;

// public class SetClimberSetpoint extends Command {
//   private final Climber m_climber;
//   private final double m_setpoint;
//   private final DoubleSupplier m_input;

//   public SetClimberSetpoint(Climber climber, double setpoint) {
//     this(climber, setpoint, () -> 0);
//   }

//   /** Creates a new SetClimberSetpoint. */
//   public SetClimberSetpoint(Climber climber, double setpoint, DoubleSupplier input) {
//     m_climber = climber;
//     m_setpoint = setpoint;
//     m_input = input;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_climber);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_climber.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP);
//     m_climber.setDesiredPositionMeters(m_setpoint);
//     m_climber.setUserSetpoint(true);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_climber.setDesiredPositionMeters(m_setpoint);

//     double joystickDeadbandOutput = MathUtil.applyDeadband(m_input.getAsDouble(), 0.1);
//     m_climber.setJoystickY(-joystickDeadbandOutput);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_climber.setUserSetpoint(false);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
