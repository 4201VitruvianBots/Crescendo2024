// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.SysIdUtils;

public class SwerveTurnQuasistatic extends SequentialCommandGroup {
  /** Creates a new SwerveTurnQuasistatic. */
  public SwerveTurnQuasistatic(
      CommandSwerveDrivetrain swerveDrive, SysIdRoutine.Direction direction) {
    var routine = SysIdUtils.getSwerveTurnRoutine();

    Command sysidCommand = routine.quasistatic(direction);

    var point = new SwerveRequest.PointWheelsAt();

    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();

    addCommands(
        new InstantCommand(
            () -> swerveDrive.applyRequest(() -> point.withModuleDirection(new Rotation2d())),
            swerveDrive),
        new WaitCommand(1),
        sysidCommand
            .withTimeout(8)
            .andThen(() -> swerveDrive.setControl(stopRequest), swerveDrive));
  }
}
