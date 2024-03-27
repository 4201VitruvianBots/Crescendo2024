// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.VISION;
import frc.robot.constants.VISION.TRACKING_STATE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

public class AutoSetTrackingState extends InstantCommand {
  private final CommandSwerveDrivetrain m_swerveDrive;

  private final VISION.TRACKING_STATE m_state;
  private final Intake m_intake;
  private final boolean m_isSOTM;

  public AutoSetTrackingState(CommandSwerveDrivetrain swerveDrive, Intake intake, VISION.TRACKING_STATE state, boolean isSOTM) {
    m_swerveDrive = swerveDrive;
    m_state = state;
    m_intake = intake;
    m_isSOTM = isSOTM;
  }
   public AutoSetTrackingState(CommandSwerveDrivetrain swerveDrive, Intake intake, VISION.TRACKING_STATE state) {
    m_swerveDrive = swerveDrive;
    m_state = state;
    m_intake = intake;
    m_isSOTM = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
     if (m_isSOTM == true) {
      if (m_intake.checkEitherIntakeSensorActive()) {
            m_swerveDrive.setTrackingState(TRACKING_STATE.SPEAKER);

      }
      else {
        // new WaitCommand(0.75);
        //  PPHolonomicDriveController.reset(m_swerveDrive.getState().Pose ,getChassisSpeed());
        m_swerveDrive.setTrackingState(m_state);
      }

    }


    else {
    m_swerveDrive.setTrackingState(m_state);
    }
  }
}
