// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.AutoScore;
import frc.robot.constants.AMP.AMP_STATE;
import frc.robot.constants.INTAKE.INTAKE_STATE;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreTest extends SequentialCommandGroup {
  /** Creates a new AutoScoreTest. */
  public AutoScoreTest(
      Shooter shooter,
      AmpShooter ampShooter,
      Intake intake,
      double AmpPercentOutput,
      double RPMOutput,
      double FrontIntakeAmpPercentOutput,
      Double BackIntakeAmpPercentOutput) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutoScore(
            shooter,
            ampShooter,
            intake,
            AMP_STATE.INTAKING.get(),
            RPM_SETPOINT.SPEAKER.get(),
            INTAKE_STATE.FRONT_ROLLER_INTAKING.get(),
            INTAKE_STATE.BACK_ROLLER_INTAKING.get()));
  }
}
