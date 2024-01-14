// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.ModuleMap.MODULE_POSITION;
import frc.robot.utils.SysidUtils;

public class SwerveTurnDynamic extends SequentialCommandGroup {
  /** Creates a new SwerveTurnDynamic. */
  public SwerveTurnDynamic(SwerveDrive swerveDrive, SysIdRoutine.Direction direction) {
    var routine = SysidUtils.getSwerveTurnRoutine();
    MODULE_POSITION position = MODULE_POSITION.FRONT_LEFT;
    var module = swerveDrive.getSwerveModule(position);

    Command sysidCommand = routine.dynamic(direction);

    addCommands(
        new InstantCommand(() -> module.setDesiredState(new SwerveModuleState(), false), module),
        new WaitCommand(1),
        sysidCommand
            .withTimeout(8)
            .andThen(() -> module.setDesiredState(new SwerveModuleState(), false), module));
  }
}
