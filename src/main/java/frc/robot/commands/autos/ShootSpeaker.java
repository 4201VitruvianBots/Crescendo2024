package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.commands.shooter.AutoSetRPMSetpoint;
import frc.robot.commands.uptake.RunUptake;
import frc.robot.constants.FLYWHEEL.FLYWHEEL_STATE;
import frc.robot.constants.FLYWHEEL.WAIT;

public class ShootSpeaker extends SequentialCommandGroup {
    
    public ShootSpeaker(Shooter shooter, Uptake uptake) {
        
    addCommands(
        new AutoSetRPMSetpoint(shooter, FLYWHEEL_STATE.SPEAKER),
        new WaitCommand(WAIT.WAIT_FOR_FLYWHEEL_SETPOINT.get()),
        new RunUptake(uptake, 0.5));
    }
}
