// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraightChoreoTest extends SequentialCommandGroup {
  /** Creates a new DriveStraightTest. */
  public DriveStraightChoreoTest(SwerveDrive swerveDrive, FieldSim fieldSim) {

    PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("DriveStraightTest");

    var m_ppCommand = TrajectoryUtils.generatePPHolonomicCommand(swerveDrive, path, path.getGlobalConstraints().getMaxVelocityMps(), false);

    SwerveModuleState[] states = {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
    };

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
            new SetSwerveOdometry(swerveDrive, path.getPreviewStartingHolonomicPose(), fieldSim),
            new InstantCommand(() -> swerveDrive.setSwerveModuleStates(states, false)).alongWith(new WaitCommand(1)),
            m_ppCommand.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
