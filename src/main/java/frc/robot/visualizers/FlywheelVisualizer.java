// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visualizers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.ARM;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.INTAKE;
import frc.robot.constants.LED;
import frc.robot.constants.ROBOT;
import frc.robot.constants.SHOOTER;
import frc.robot.constants.VISION;
import frc.robot.subsystems.*;

/** A class to visualize the state of all mechanisms on the robot. */
public class FlywheelVisualizer {
  Intake m_intake;
  Shooter m_shooter;
  AmpShooter m_ampShooter;
  Arm m_arm;
  Climber m_climber;
  Vision m_vision;
  LEDSubsystem m_led;

  double FlywheelSize = Units.inchesToMeters((3.4 * Math.PI) / 8);

  Mechanism2d m_mech2d = new Mechanism2d(ROBOT.drivebaseLength * 2, ROBOT.drivebaseLength * 2);

  MechanismRoot2d m_drivebaseRoot2d =
      m_mech2d.getRoot("Drivebase", ROBOT.drivebaseLength * 0.5, ROBOT.drivebaseWidth * 0.5);

  MechanismRoot2d m_goalBaseRoot2d =
      m_mech2d.getRoot("GoalBase", Units.inchesToMeters(126.664), ROBOT.drivebaseWidth * 0.5);
  MechanismRoot2d m_climberRoot2d =
      m_mech2d.getRoot(
          "Climber",
          ROBOT.drivebaseLength * 0.5 + CLIMBER.kDistanceFromIntake,
          ROBOT.drivebaseWidth * 0.5);
  MechanismRoot2d m_shooterRoot2d =
      m_mech2d.getRoot(
          "Shooter",
          ROBOT.drivebaseLength * 0.5 + SHOOTER.kDistanceFromIntake,
          ROBOT.drivebaseWidth * 0.5);

  MechanismLigament2d m_drivebase2d =
      m_drivebaseRoot2d.append(new MechanismLigament2d("Drivebase", ROBOT.drivebaseLength, 0));
  MechanismLigament2d m_limelightA2d =
      m_drivebaseRoot2d.append(
          new MechanismLigament2d(
              "LimelightA", VISION.aprilTagLimelightCameraADistanceFromGroundZ, 90));
  MechanismLigament2d m_limelightB2d =
      m_drivebaseRoot2d.append(
          new MechanismLigament2d(
              "LimelightB", VISION.aprilTagLimelightCameraBDistanceFromGroundZ, 90));
  MechanismLigament2d m_goalBase2d =
      m_goalBaseRoot2d.append(new MechanismLigament2d("GoalBase", Units.inchesToMeters(99.131), 0));
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

  MechanismLigament2d m_bottomFlywheel =
      m_mech2d
          .getRoot(
              "pivotPoint",
              ROBOT.drivebaseLength * 0.5 + SHOOTER.kDistanceFromIntake + Units.inchesToMeters(2),
              (ROBOT.drivebaseWidth * 0.5) + Units.inchesToMeters(11.3125))
          .append(
              new MechanismLigament2d(
                  "arm", Units.inchesToMeters(1.7), 0, 0, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d side1 =
      m_bottomFlywheel.append(
          new MechanismLigament2d("side1", FlywheelSize, 112.5, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d side2 =
      side1.append(
          new MechanismLigament2d("side2", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d side3 =
      side2.append(
          new MechanismLigament2d("side3", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d side4 =
      side3.append(
          new MechanismLigament2d("side4", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d side5 =
      side4.append(
          new MechanismLigament2d("side5", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d side6 =
      side5.append(
          new MechanismLigament2d("side6", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d side7 =
      side6.append(
          new MechanismLigament2d("side7", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d side8 =
      side7.append(
          new MechanismLigament2d("side8", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));

  // Velocity

  // Position
  MechanismLigament2d m_topFlywheel =
      m_mech2d
          .getRoot(
              "UpperpivotPoint",
              ROBOT.drivebaseLength * 0.5 + SHOOTER.kDistanceFromIntake - Units.inchesToMeters(2),
              (ROBOT.drivebaseWidth * 0.5) + Units.inchesToMeters(15.3125))
          .append(
              new MechanismLigament2d(
                  "Upperarm", Units.inchesToMeters(1.7), 0, 0, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d Upperside1 =
      m_topFlywheel.append(
          new MechanismLigament2d(
              "Upperside1", FlywheelSize, 112.5, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d Upperside2 =
      Upperside1.append(
          new MechanismLigament2d(
              "Upperside2", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d Upperside3 =
      Upperside2.append(
          new MechanismLigament2d(
              "Upperside3", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d Upperside4 =
      Upperside3.append(
          new MechanismLigament2d(
              "Upperside4", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d Upperside5 =
      Upperside4.append(
          new MechanismLigament2d(
              "Upperside5", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d Upperside6 =
      Upperside5.append(
          new MechanismLigament2d(
              "Upperside6", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d Upperside7 =
      Upperside6.append(
          new MechanismLigament2d(
              "Upperside7", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));
  MechanismLigament2d Upperside8 =
      Upperside7.append(
          new MechanismLigament2d(
              "Upperside8", FlywheelSize, 45, 3, new Color8Bit(Color.kDimGray)));

  Color8Bit m_drivebase2d_originalColor,
      m_limelight2d_originalColor,
      m_intake2d_originalColor,
      m_climber2d_originalColor,
      m_climberHook1_2d_originalColor,
      m_climberHook2_2d_originalColor,
      m_shooter2d_originalColor,
      m_arm2d_originalColor,
      m_ampShooter2d_originalColor;

  public FlywheelVisualizer() {
    m_drivebase2d.setColor(new Color8Bit(235, 137, 52));
    m_goalBase2d.setColor(new Color8Bit(235, 235, 52));
    m_limelightA2d.setColor(new Color8Bit(45, 235, 60));
    m_limelightB2d.setColor(new Color8Bit(45, 235, 60));
    m_intake2d.setColor(new Color8Bit(235, 229, 52));
    m_climber2d.setColor(new Color8Bit(52, 212, 235));
    m_climberHook1_2d.setColor(new Color8Bit(52, 212, 235));
    m_climberHook2_2d.setColor(new Color8Bit(52, 212, 235));
    m_shooter2d.setColor(new Color8Bit(189, 189, 189));
    m_arm2d.setColor(new Color8Bit(235, 137, 52));
    m_ampShooter2d.setColor(new Color8Bit(235, 205, 52));

    m_drivebase2d_originalColor = m_drivebase2d.getColor();
    m_limelight2d_originalColor = m_limelightA2d.getColor();
    m_limelight2d_originalColor = m_limelightB2d.getColor();
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

  /* Function to visualize the speed of a particular motor. */

  /* Function to visualize the state of a limelight. */

  public void updateShooter() {
    // updateMotorColor(m_shooter2d, m_shooter.getRpmFollower(), m_shooter2d_originalColor);

    m_bottomFlywheel.setAngle(
        m_bottomFlywheel.getAngle() - 360 * m_shooter.getRpmMaster() / 60 * 0.2);
    m_topFlywheel.setAngle(m_topFlywheel.getAngle() + 360 * m_shooter.getRpmFollower() / 60 * 0.2);
  }

  public void updateLED() {
    m_led2d.setColor(m_led.getColor());
  }

  public void periodic() {
    if (m_shooter != null) updateShooter();
    if (m_led != null) updateLED();
  }
}
