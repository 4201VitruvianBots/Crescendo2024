// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.TrajectoryUtils;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class frourpeiceNear extends SequentialCommandGroup {
  /** Creates a new DriveStraightTest. */
  public frourpeiceNear(CommandSwerveDrivetrain swerveDrive, FieldSim fieldSim) {

    PathPlannerPath path1 = PathPlannerPath.fromPathFile("fourpiecept1");
var m_ppCommand1 = TrajectoryUtils.generatePPHolonomicCommand(swerveDrive, path1, path1.getGlobalConstraints().getMaxVelocityMps(), false);
PathPlannerPath path2 = PathPlannerPath.fromPathFile( "fourpiecept2");
        var m_ppCommand2 = TrajectoryUtils.generatePPHolonomicCommand(swerveDrive, path2, path2.getGlobalConstraints().getMaxVelocityMps(), false);
    PathPlannerPath path3 = PathPlannerPath.fromPathFile( "fourpiecept3");
        var m_ppCommand3 = TrajectoryUtils.generatePPHolonomicCommand(swerveDrive, path3, path3.getGlobalConstraints().getMaxVelocityMps(), false);
        PathPlannerPath path4 = PathPlannerPath.fromPathFile( "fourpiecept4");
        var m_ppCommand4 = TrajectoryUtils.generatePPHolonomicCommand(swerveDrive, path4, path4.getGlobalConstraints().getMaxVelocityMps(), false);
        PathPlannerPath path5 = PathPlannerPath.fromPathFile( "fourpiecept5");
        var m_ppCommand5 = TrajectoryUtils.generatePPHolonomicCommand(swerveDrive, path5, path5.getGlobalConstraints().getMaxVelocityMps(), false);

    var point = new SwerveRequest.PointWheelsAt();
    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
     ArrayList<PathPlannerPath> pathsList = new ArrayList<>();
pathsList.add(path1);
pathsList.add(path2);
pathsList.add(path3);
pathsList.add(path4);
pathsList.add(path5);
    
    addCommands(
        new PlotAutoPath(fieldSim, "AutoExample", pathsList),
        new InstantCommand(
            () -> swerveDrive.seedFieldRelative(path1.getPreviewStartingHolonomicPose())),
        new InstantCommand(
                () -> swerveDrive.applyRequest(() -> point.withModuleDirection(new Rotation2d())),
                swerveDrive)
            .alongWith(new WaitCommand(1)),
        m_ppCommand1,m_ppCommand2,m_ppCommand3,m_ppCommand4,
        m_ppCommand5.andThen(() -> swerveDrive.setControl(stopRequest)));
  }
}
