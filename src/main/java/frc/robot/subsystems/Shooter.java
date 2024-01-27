// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.FLYWHEEL;

public class Shooter extends SubsystemBase {
  private final TalonFX flywheelmotor1 = new TalonFX(CAN.flywheel1);
  private final TalonFX flywheelmotor2 = new TalonFX(CAN.flywheel2);
  private double m_rpm;
  private double m_headingOffset;
  private double flywheelRPMRatio = 1.0;

  // private final ConfigFactoryDefault configSelectedFeedbackSensor = new Config
  /* Creates a new Intake. */
  public Shooter() {
    // flywheel motor 1
    flywheelmotor1.setInverted(true);
    // flywheel motor 2
    flywheelmotor2.setInverted(false);
  }

  // values that we set
  public void setRPM(double m_rpm) {
    flywheelmotor1.set(m_rpm);
    flywheelmotor2.set(m_rpm * flywheelRPMRatio);
  }

  public double ShootNStrafeAngle(Pose2d RobotPose, ChassisSpeeds RobotVelocity) {
    return Math.atan2(
    (FLYWHEEL.NoteVelocity * Math.sin(this.shootangle(RobotPose)) - RobotVelocity.vyMetersPerSecond),
    (FLYWHEEL.NoteVelocity * Math.cos(this.shootangle(RobotPose)) - RobotVelocity.vxMetersPerSecond));
  }
  private double shootangle(Pose2d RobotPose) {
    if (Controls.IsBlueAllaince())
    {
    return (
        Math.atan2((FLYWHEEL.SPEAKER.BlueSpeakerTLY - RobotPose.getY()), (FLYWHEEL.SPEAKER.BlueSpeakerTLX - RobotPose.getX())) 
      + Math.atan2((FLYWHEEL.SPEAKER.BlueSpeakerTRY - RobotPose.getY()), (FLYWHEEL.SPEAKER.BlueSpeakerTRX - RobotPose.getX()))
      + Math.atan2((FLYWHEEL.SPEAKER.BlueSpeakerBLY - RobotPose.getY()), (FLYWHEEL.SPEAKER.BlueSpeakerBLX - RobotPose.getX()))
      + Math.atan2((FLYWHEEL.SPEAKER.BlueSpeakerBRY - RobotPose.getY()), (FLYWHEEL.SPEAKER.BlueSpeakerBRX - RobotPose.getX())))/4;
    }else
    {
    return (
        Math.atan2((FLYWHEEL.SPEAKER.RedSpeakerTLY - RobotPose.getY()), (FLYWHEEL.SPEAKER.RedSpeakerTLX - RobotPose.getX())) 
      + Math.atan2((FLYWHEEL.SPEAKER.RedSpeakerTRY - RobotPose.getY()), (FLYWHEEL.SPEAKER.RedSpeakerTRX - RobotPose.getX()))
      + Math.atan2((FLYWHEEL.SPEAKER.RedSpeakerBLY - RobotPose.getY()), (FLYWHEEL.SPEAKER.RedSpeakerBLX - RobotPose.getX()))
      + Math.atan2((FLYWHEEL.SPEAKER.RedSpeakerBRY - RobotPose.getY()), (FLYWHEEL.SPEAKER.RedSpeakerBRX - RobotPose.getX())))/4;
    }
  }

  // values that we are pulling
  public double getRPM1() {
    return m_rpm;
  }

  public double getRPM2() {
    return m_rpm * flywheelRPMRatio;
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("RPM1", this.getRPM1());
    SmartDashboard.putNumber("RPM2", this.getRPM2());
  }

  @Override
  public void periodic() {
    this.updateShuffleboard();
  }
}
