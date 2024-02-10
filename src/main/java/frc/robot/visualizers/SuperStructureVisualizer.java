// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visualizers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.ARM;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.FLYWHEEL;
import frc.robot.constants.INTAKE;
import frc.robot.constants.LED;
import frc.robot.constants.ROBOT;
import frc.robot.constants.VISION;
import frc.robot.subsystems.*;

/** A class to visualize the state of all mechanisms on the robot. */
public class SuperStructureVisualizer {
  Intake m_intake;
  Shooter m_shooter;
  AmpShooter m_ampShooter;
  Arm m_arm;
  Climber m_climber;
  Vision m_vision;
  LEDSubsystem m_led;

  Mechanism2d m_mech2d = new Mechanism2d(ROBOT.drivebaseLength * 2, ROBOT.drivebaseLength * 2);

  MechanismRoot2d m_drivebaseRoot2d =
      m_mech2d.getRoot("Drivebase", ROBOT.drivebaseLength * 0.5, ROBOT.drivebaseWidth * 0.5);
  MechanismRoot2d m_climberRoot2d =
      m_mech2d.getRoot(
          "Climber",
          ROBOT.drivebaseLength * 0.5 + CLIMBER.kDistanceFromIntake,
          ROBOT.drivebaseWidth * 0.5);
  MechanismRoot2d m_shooterRoot2d =
      m_mech2d.getRoot(
          "Shooter",
          ROBOT.drivebaseLength * 0.5 + FLYWHEEL.kDistanceFromIntake,
          ROBOT.drivebaseWidth * 0.5);

  MechanismLigament2d m_drivebase2d =
      m_drivebaseRoot2d.append(new MechanismLigament2d("Drivebase", ROBOT.drivebaseLength, 0));
  MechanismLigament2d m_limelight2d =
      m_drivebaseRoot2d.append(new MechanismLigament2d("Limelight", VISION.limelightHeight, 90));
  MechanismLigament2d m_intake2d =
      m_drivebaseRoot2d.append(new MechanismLigament2d("Intake", INTAKE.intakeLength, 0));

  MechanismLigament2d m_led2d =
      m_shooterRoot2d.append(new MechanismLigament2d("LED", LED.LEDstripLength, 70));
  MechanismLigament2d m_shooter2d =
      m_shooterRoot2d.append(new MechanismLigament2d("Shooter", Units.inchesToMeters(22), 90));
  MechanismLigament2d m_arm2d =
      m_shooter2d.append(
          new MechanismLigament2d(
              "Arm", ARM.length, ARM.startingAngleDegrees + ARM.mountingAngleDegrees));
  MechanismLigament2d m_ampShooter2d =
      m_arm2d.append(new MechanismLigament2d("Amp Shooter", Units.inchesToMeters(6), 0));

  MechanismLigament2d m_climber2d =
      m_climberRoot2d.append(new MechanismLigament2d("Climber", CLIMBER.kUnextendedLength, 90));
  MechanismLigament2d m_climberHook1_2d =
      m_climber2d.append(new MechanismLigament2d("Hook 1", Units.inchesToMeters(3), -90));
  MechanismLigament2d m_climberHook2_2d =
      m_climberHook1_2d.append(new MechanismLigament2d("Hook 2", Units.inchesToMeters(3), -90));

  Color8Bit m_drivebase2d_originalColor,
      m_limelight2d_originalColor,
      m_intake2d_originalColor,
      m_climber2d_originalColor,
      m_climberHook1_2d_originalColor,
      m_climberHook2_2d_originalColor,
      m_shooter2d_originalColor,
      m_arm2d_originalColor,
      m_ampShooter2d_originalColor;

  public SuperStructureVisualizer() {
    m_drivebase2d.setColor(new Color8Bit(235, 137, 52));
    m_limelight2d.setColor(new Color8Bit(53, 235, 52));
    m_intake2d.setColor(new Color8Bit(235, 229, 52));
    m_climber2d.setColor(new Color8Bit(52, 212, 235));
    m_climberHook1_2d.setColor(new Color8Bit(52, 212, 235));
    m_climberHook2_2d.setColor(new Color8Bit(52, 212, 235));
    m_shooter2d.setColor(new Color8Bit(189, 189, 189));
    m_arm2d.setColor(new Color8Bit(235, 137, 52));
    m_ampShooter2d.setColor(new Color8Bit(235, 205, 52));

    m_drivebase2d_originalColor = m_drivebase2d.getColor();
    m_limelight2d_originalColor = m_limelight2d.getColor();
    m_intake2d_originalColor = m_intake2d.getColor();
    m_climber2d_originalColor = m_climber2d.getColor();
    m_climberHook1_2d_originalColor = m_climberHook1_2d.getColor();
    m_climberHook2_2d_originalColor = m_climberHook2_2d.getColor();
    m_shooter2d_originalColor = m_shooter2d.getColor();
    m_arm2d_originalColor = m_arm2d.getColor();
    m_ampShooter2d_originalColor = m_ampShooter2d.getColor();

    SmartDashboard.putData("SuperStructure Sim", m_mech2d);
  }

  public void registerIntake(Intake intake) {
    m_intake = intake;
  }

  public void registerShooter(Shooter shooter) {
    m_shooter = shooter;
  }

  public void registerAmpShooter(AmpShooter ampShooter) {
    m_ampShooter = ampShooter;
  }

  public void registerArm(Arm arm) {
    m_arm = arm;
  }

  public void registerClimber(Climber climber) {
    m_climber = climber;
  }

  public void registerVision(Vision vision) {
    m_vision = vision;
  }

  public void registerLedSubsystem(LEDSubsystem led) {
    m_led = led;
  }

  /* Function to visualize the speed of a particular motor. */
  public void updateMotorColor(
      MechanismLigament2d ligament, double motorSpeed, Color8Bit originalColor) {
    double deltaBrightness = Math.abs(motorSpeed) * 75;

    Color8Bit newColor =
        new Color8Bit(
            originalColor.red + (int) deltaBrightness,
            originalColor.green + (int) deltaBrightness,
            originalColor.blue + (int) deltaBrightness);

    ligament.setColor(newColor);
  }

  /* Function to visualize the state of a limelight. */
  public void updateLimelightColor(
      MechanismLigament2d ligament, boolean isActive, Color8Bit originalColor) {

    Color8Bit newColor =
        new Color8Bit(
            originalColor.red + (isActive ? 75 : 0),
            originalColor.green + (isActive ? 75 : 0),
            originalColor.blue + (isActive ? 75 : 0));

    ligament.setColor(newColor);
  }

  public void updateIntake() {
    updateMotorColor(m_intake2d, m_intake.getSpeed(), m_intake2d_originalColor);
  }

  public void updateShooter() {
    updateMotorColor(m_shooter2d, m_shooter.getRPMFollower(), m_shooter2d_originalColor);
  }

  public void updateAmpShooter() {
    updateMotorColor(m_ampShooter2d, m_ampShooter.getVelocity(), m_ampShooter2d_originalColor);
  }

  public void updateArm() {
    updateMotorColor(m_arm2d, m_arm.getPercentOutput(), m_arm2d_originalColor);
    m_arm2d.setAngle(90 - m_arm.getAngleDegrees());
  }

  public void updateClimber() {
    updateMotorColor(m_climber2d, m_climber.getPercentOutput(), m_climber2d_originalColor);
    updateMotorColor(
        m_climberHook1_2d, m_climber.getPercentOutput(), m_climberHook1_2d_originalColor);
    updateMotorColor(
        m_climberHook2_2d, m_climber.getPercentOutput(), m_climberHook2_2d_originalColor);
    m_climber2d.setLength(CLIMBER.kUnextendedLength + m_climber.getHeightMeters());
  }

  public void updateLimelight() {
    updateLimelightColor(m_limelight2d, m_vision.isCameraConnected(), m_limelight2d_originalColor);
  }

  public void updateLED() {
    m_led2d.setColor(m_led.getColor());
  }

  public void periodic() {
    if (m_intake != null) updateIntake();
    if (m_shooter != null) updateShooter();
    if (m_ampShooter != null) updateAmpShooter();
    if (m_arm != null) updateArm();
    if (m_climber != null) updateClimber();
    if (m_vision != null) updateLimelight();
    if (m_led != null) updateLED();
  }
}
