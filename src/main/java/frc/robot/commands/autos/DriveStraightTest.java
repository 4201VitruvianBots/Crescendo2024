// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.TrajectoryUtils;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraightTest extends SequentialCommandGroup {
  /** Creates a new DriveStraightTest. */
  public DriveStraightTest(CommandSwerveDrivetrain swerveDrive) {

    PathPlannerPath path = PathPlannerPath.fromPathFile("Drivefowardtest");

    var m_ppCommand = TrajectoryUtils.generatePPHolonomicCommand(swerveDrive, path, 1, false);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(m_ppCommand.andThen(() -> swerveDrive.setChassisSpeed(new ChassisSpeeds())));
  }
}