// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceFar extends SequentialCommandGroup {
  /** Creates a new ThreePieceFar. */
  public ThreePieceFar(
      CommandSwerveDrivetrain swerveDrive,
      FieldSim fieldSim,
      Intake intake,
      AmpShooter ampShooter,
      Shooter shooter) {
    String[] pathFiles = {
      "3Piece2Pt1", "3Piece2Pt2", "3Piece2Pt3", "3Piece2Pt4",
    };
    var pathFactory = new AutoFactory.PathFactory(swerveDrive, pathFiles);

    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();

    addCommands(
        pathFactory.createAutoInit(),
        pathFactory.getNextPathCommand(),
        pathFactory.getNextPathCommand(),
        pathFactory.getNextPathCommand(),
        pathFactory.getNextPathCommand());
  }
}
