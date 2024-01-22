// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.SysidUtils;

public class SwerveTurnDynamic extends SequentialCommandGroup {
  /** Creates a new SwerveTurnDynamic. */
  public SwerveTurnDynamic(CommandSwerveDrivetrain swerveDrive, SysIdRoutine.Direction direction) {
    var routine = SysidUtils.getSwerveTurnRoutine();

    Command sysidCommand = routine.dynamic(direction);

    var point = new SwerveRequest.PointWheelsAt();

    addCommands(
        new InstantCommand(
            () -> swerveDrive.applyRequest(() -> point.withModuleDirection(new Rotation2d())),
            swerveDrive),
        new WaitCommand(1),
        sysidCommand
            .withTimeout(8)
            .andThen(() -> swerveDrive.setChassisSpeed(new ChassisSpeeds()), swerveDrive));
  }
}
