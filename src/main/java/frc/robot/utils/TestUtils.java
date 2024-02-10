// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.ROBOT;

/** Add your docs here. */
public class TestUtils {
    private static class PrintSuccess extends InstantCommand {
        public PrintSuccess(String commandName) {
            super(() -> System.out.println(ROBOT.ANSI_GREEN + "✓  " + commandName + " ran successfully" + ROBOT.ANSI_RESET));
        }
    }
    private static class PrintFailure extends InstantCommand {
        public PrintFailure(String commandName, double timeout) {
            super(() -> System.out.println(ROBOT.ANSI_RED + "✗  " + commandName + " failed after " + timeout + " seconds" + ROBOT.ANSI_RESET));
        }
    }
    
    private static class PrintConfirmation extends InstantCommand {
        public PrintConfirmation(String commandName) {
            super(() -> System.out.println(ROBOT.ANSI_YELLOW + "⚠  Do you want to run " + commandName + "? (Press A to confirm, B to deny)" + ROBOT.ANSI_RESET));
        }
    }
    
    private static class PrintStart extends InstantCommand {
        public PrintStart(String commandName) {
            super(() -> System.out.println(ROBOT.ANSI_YELLOW + "⚠  Running command " + commandName + "..." + ROBOT.ANSI_RESET));
        }
    }
    
    
    public static ParallelRaceGroup runCheck(Command command, double timeout, BooleanSupplier aButton, BooleanSupplier bButton) {
        ParallelDeadlineGroup runCommand = new ParallelDeadlineGroup(new WaitCommand(timeout).andThen(new PrintFailure(command.getName(), timeout)), command.andThen(new PrintSuccess(command.getName())));
        
        // Print confirmation to ask the user whether they want to run the command, and then run the command if the user presses the confirm button
        // Run PrintConfirmation and then run the command if the user presses the confirm button, or deny the command if the user presses the deny button
        return new ParallelRaceGroup(new PrintConfirmation(command.getName()).andThen(new WaitUntilCommand(aButton)).andThen(new PrintStart(command.getName())).andThen(runCommand), new WaitUntilCommand(bButton).andThen(new PrintFailure(command.getName(), 0)));
    }
}
