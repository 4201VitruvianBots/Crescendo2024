// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.system.*;
import frc.robot.subsystems.*;
import frc.robot.constants.ROBOT;
import frc.robot.utils.SystemCheckUtils;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemCheck extends SequentialCommandGroup {
  /** Creates a new SystemCheck. */
public SystemCheck(AmpShooter ampShooter, Arm arm, Climber climber, Intake intake, LEDSubsystem ledSubsystem, Shooter shooter, CommandSwerveDrivetrain swerveDrivetrain, Vision vision, BooleanSupplier aButton, BooleanSupplier bButton) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new PrintCommand(ROBOT.ANSI_CYAN + "Welcome to Team 4201's System Check Interface!" + ROBOT.ANSI_RESET),
        new PrintCommand(ROBOT.ANSI_CYAN + "This interface will check all subsystems to ensure they are functioning properly." + ROBOT.ANSI_RESET),
        new PrintCommand(ROBOT.ANSI_CYAN + "Please ensure that the robot and team members are in safe locations and that all subsystems are ready to be tested." + ROBOT.ANSI_RESET),
        
        // Check subsystems that don't require human interaction, i.e. LEDSubsystem, Vision, and Swerve
        SystemCheckUtils.runCheck(new CheckSwerve(swerveDrivetrain), 5, aButton, bButton),
        SystemCheckUtils.runCheck(new CheckLEDSubsystem(ledSubsystem), 5, aButton, bButton),
        SystemCheckUtils.runCheck(new CheckVision(vision), 5, aButton, bButton),
        
        // Check subsystems that require human interaction to test full functionality, i.e. Arm, AmpShooter, Intake, Shooter, and Climber
        SystemCheckUtils.runCheck(new CheckArm(arm), 5, aButton, bButton),
        SystemCheckUtils.runCheck(new CheckAmpShooter(ampShooter), 5, aButton, bButton),
        SystemCheckUtils.runCheck(new CheckIntake(intake), 5, aButton, bButton),
        SystemCheckUtils.runCheck(new CheckShooter(shooter), 5, aButton, bButton),
        SystemCheckUtils.runCheck(new CheckClimber(climber), 5, aButton, bButton)
    );
}
}
