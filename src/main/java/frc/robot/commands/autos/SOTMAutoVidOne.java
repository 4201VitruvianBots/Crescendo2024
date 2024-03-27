// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.shooter.AutoSetRPMSetpoint;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SOTMAutoVidOne extends SequentialCommandGroup {
  /** Creates a new ThreePieceFar. */
  public SOTMAutoVidOne(
      CommandSwerveDrivetrain swerveDrive, Intake intake, AmpShooter ampShooter, Shooter shooter) {
    String[] pathFiles = {"SOTMAutoVid-2"};
    var pathFactory = new AutoFactory.PathFactory(swerveDrive, pathFiles);
    var flywheelCommandContinuous = new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.AUTO_RPM.get());

    var shooterFactory = new AutoFactory.ShootFactory(intake, ampShooter, shooter);

    // TODO: After testing vision, make shoot command check for angle to speaker with a tolerance
    // value
    // TODO: Need to think about how long to aim before shooting?
    addCommands(
        pathFactory.createAutoInit(),
        new ParallelDeadlineGroup(
            pathFactory.getNextPathCommand(),
            flywheelCommandContinuous,
            new SequentialCommandGroup(
                new WaitCommand(1.75), shooterFactory.generateShootCommand())));
  }
}
