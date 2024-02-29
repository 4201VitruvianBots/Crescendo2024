// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraightChoreoTest extends SequentialCommandGroup {
  /** Creates a new DriveStraightTest. */
  public DriveStraightChoreoTest(CommandSwerveDrivetrain swerveDrive, FieldSim fieldSim) {

    //    var traj = Choreo.getTrajectory("44r4r");

    //    var m_ppCommand = TrajectoryUtils.generateChoreoCommand(swerveDrive, traj, 1.0, false);

    var point = new SwerveRequest.PointWheelsAt();
    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //    addCommands(
    //        // new PlotAutoPath(fieldSim, "44r4r", traj),
    //        new InstantCommand(() -> swerveDrive.seedFieldRelative(traj.getInitialPose())),
    //        new InstantCommand(
    //                () -> swerveDrive.applyRequest(() -> point.withModuleDirection(new
    // Rotation2d())),
    //                swerveDrive)
    //            .alongWith(new WaitCommand(1)),
    //        m_ppCommand.andThen(() -> swerveDrive.setControl(stopRequest)));
  }
}
