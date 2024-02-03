// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.TrajectoryUtils;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraightPathplannerTest extends SequentialCommandGroup {
  /** Creates a new DriveStraightTest. */
  public DriveStraightPathplannerTest(CommandSwerveDrivetrain swerveDrive, FieldSim fieldSim) {

    PathPlannerPath path = PathPlannerPath.fromPathFile("Drivefowardtest");

    var m_ppCommand = TrajectoryUtils.generatePPHolonomicCommand(swerveDrive, path, 1.0, false);

    var point = new SwerveRequest.PointWheelsAt();
    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new PlotAutoPath(fieldSim, "Drivefowardtest", path),
        new InstantCommand(
            () -> swerveDrive.seedFieldRelative(path.getPreviewStartingHolonomicPose())),
        new InstantCommand(
                () -> swerveDrive.applyRequest(() -> point.withModuleDirection(new Rotation2d())),
                swerveDrive)
            .alongWith(new WaitCommand(1)),
        m_ppCommand.andThen(() -> swerveDrive.setControl(stopRequest)));
  }
}
