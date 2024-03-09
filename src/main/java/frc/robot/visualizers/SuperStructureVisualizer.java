// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visualizers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.INTAKE;
import frc.robot.constants.LED;
import frc.robot.constants.ROBOT;
import frc.robot.constants.SHOOTER;
import frc.robot.constants.VISION;
import frc.robot.subsystems.*;
import java.util.ArrayList;

/** A class to visualize the state of all mechanisms on the robot. */
public class SuperStructureVisualizer implements AutoCloseable {
  Intake m_intake;
  Shooter m_shooter;
  AmpShooter m_ampShooter;
  Arm m_arm;
  Climber m_climber;
  Vision m_vision;
  LEDSubsystem m_led;

  private final double startPointX = 0.28;
  private final double startPointY = 0.2;

  private final Mechanism2d m_mech2d =
      new Mechanism2d(ROBOT.drivebaseLength * 2, ROBOT.drivebaseLength * 2);

  private final MechanismRoot2d m_drivebaseRoot2d =
      m_mech2d.getRoot("startRoot", startPointX, startPointY);
  private final MechanismRoot2d m_climberRoot2d =
      m_mech2d.getRoot("climberRoot", startPointX + CLIMBER.kDistanceFromIntake, startPointY);
  private final MechanismRoot2d m_climberPostRoot2d =
      m_mech2d.getRoot("climberPostRoot", startPointX + CLIMBER.kDistanceFromIntake, startPointY);
  private final MechanismRoot2d m_superStructureRoot =
      m_mech2d.getRoot("superStructureRoot", startPointX + Units.inchesToMeters(23), startPointY);

  private final MechanismLigament2d m_drivebase2d =
      m_drivebaseRoot2d.append(
          new MechanismLigament2d("drivebase", INTAKE.intakeLength + ROBOT.drivebaseLength, 0));
  private final MechanismLigament2d m_limelightA2d =
      m_drivebaseRoot2d.append(
          new MechanismLigament2d(
              "LimelightA", VISION.aprilTagLimelightCameraADistanceFromGroundZ, 90));
  private final MechanismLigament2d m_limelightB2d =
      m_drivebaseRoot2d.append(
          new MechanismLigament2d(
              "LimelightB", VISION.aprilTagLimelightCameraADistanceFromGroundZ, 0));
  private final MechanismLigament2d m_intake2d =
      m_drivebaseRoot2d.append(new MechanismLigament2d("Intake", INTAKE.intakeLength, -90));
  private final ShooterVisualizer m_intakeVisualizer = new ShooterVisualizer("intake", m_intake2d);

  private final MechanismLigament2d m_shooter2d =
      m_superStructureRoot.append(
          new MechanismLigament2d("superStructure", Units.inchesToMeters(20), 83.442));

  private final MechanismLigament2d m_led2d =
      m_shooter2d.append(new MechanismLigament2d("LED", LED.LEDstripLength, 186));

  private final ArmVisualizer m_armVisualizer = new ArmVisualizer("m_arm2d");
  private final MechanismLigament2d m_arm2d =
      m_shooter2d.append(m_armVisualizer.getJointLigament());
  private final MechanismLigament2d m_ampShooter2d =
      m_armVisualizer
          .getArmLigament()
          .append(new MechanismLigament2d("Amp Shooter", Units.inchesToMeters(8), -110));

  private final ShooterVisualizer m_ampShooterVisualizer =
      new ShooterVisualizer("flywheelAmpShooter", m_ampShooter2d);

  private final ClimberVisualizer m_climberVisualizer = new ClimberVisualizer("m_climber2d");
  private final MechanismLigament2d m_climber2d =
      m_climberRoot2d.append(m_climberVisualizer.getLigament());
  private final MechanismLigament2d m_climberPost =
      m_climberPostRoot2d.append(m_climberVisualizer.getPost());

  private final MechanismRoot2d m_bottomFlywheelRoot2d =
      m_mech2d.getRoot(
          "pivotPoint",
          startPointX + SHOOTER.kBottomFlywheelDistanceFromIntake,
          startPointY + SHOOTER.kBottomFlywheelDistanceFromDriveBase);
  private final ShooterVisualizer m_bottomFlywheelVisualizer =
      new ShooterVisualizer("flywheelArmBottom", m_bottomFlywheelRoot2d);

  // Velocity

  // Position
  private final MechanismRoot2d m_topFlywheelRoot2d =
      m_mech2d.getRoot(
          "UpperPivotPoint",
          startPointX + SHOOTER.kTopFlywheelDistanceFromIntake,
          startPointY + SHOOTER.kTopFlywheelDistanceFromDriveBase);
  private final ShooterVisualizer m_topFlywheelVisualizer =
      new ShooterVisualizer("flywheelArmTop", m_topFlywheelRoot2d);

  private final Color8Bit m_limelightA2d_originalColor,
      m_limelightB2d_originalColor,
      m_intake2d_originalColor,
      m_shooter2d_originalColor,
      m_ampShooter2d_originalColor;

  private final ArrayList<VisualizerUtils.MechanismDisplay> m_displays = new ArrayList<>();
  private ArmVisualizer m_armVisualizer2;
  private ClimberVisualizer m_climberVisualizer2;

  public SuperStructureVisualizer() {
    m_drivebase2d.setColor(new Color8Bit(235, 137, 52));
    m_limelightA2d.setColor(new Color8Bit(45, 235, 45));
    m_limelightB2d.setColor(new Color8Bit(60, 235, 60));
    m_intake2d.setColor(new Color8Bit(235, 229, 52));
    m_climber2d.setColor(new Color8Bit(52, 212, 235));
    m_shooter2d.setColor(new Color8Bit(189, 189, 189));
    m_arm2d.setColor(new Color8Bit(235, 137, 52));
    m_ampShooter2d.setColor(new Color8Bit(235, 205, 52));

    m_limelightA2d_originalColor = m_limelightA2d.getColor();
    m_limelightB2d_originalColor = m_limelightB2d.getColor();
    m_intake2d_originalColor = m_intake2d.getColor();
    m_shooter2d_originalColor = m_shooter2d.getColor();
    m_ampShooter2d_originalColor = m_ampShooter2d.getColor();

    SmartDashboard.putData("SuperStructure Sim", m_mech2d);
    if (RobotBase.isSimulation()) {
      m_armVisualizer2 = new ArmVisualizer("Arm2d");
      var armDisplay =
          new VisualizerUtils.MechanismDisplay(0.5, 0.5, m_armVisualizer2.getJointLigament());
      m_displays.add(armDisplay);

      m_climberVisualizer2 = new ClimberVisualizer("Climber2d");
      var climberDisplay =
          new VisualizerUtils.MechanismDisplay(0.5, 0.25, m_climberVisualizer2.getLigament());
      climberDisplay.addLigament(0.525, 0.25, m_climberVisualizer2.getPost());
      m_displays.add(climberDisplay);

      for (var display : m_displays) display.addSmartDashboardDisplay();
    }
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

    m_intakeVisualizer.update(m_intake.getRpm());
  }

  public void updateShooter() {
    updateMotorColor(m_shooter2d, m_shooter.getRpmFollower(), m_shooter2d_originalColor);

    m_bottomFlywheelVisualizer.update(m_shooter.getRpmMaster());
    m_topFlywheelVisualizer.update(m_shooter.getRpmFollower());
  }

  public void updateAmpShooter() {
    updateMotorColor(m_ampShooter2d, m_ampShooter.getSpeed(), m_ampShooter2d_originalColor);

    m_ampShooterVisualizer.update(m_ampShooter.getRpm());
  }

  public void updateArm() {
    m_armVisualizer.update(m_arm.getCurrentAngle() + 170, m_arm.getPercentOutput());
    if (m_armVisualizer2 != null)
      m_armVisualizer2.update(m_arm.getCurrentAngle(), m_arm.getPercentOutput());
  }

  public void updateClimber() {

    m_climberVisualizer.update(m_climber.getHeightMeters(), m_climber.getPercentOutput());
    if (m_climberVisualizer2 != null)
      m_climberVisualizer2.update(m_climber.getHeightMeters(), m_climber.getPercentOutput());
  }

  public void updateLimelights() {
    // updateLimelightColor(
    //     m_limelightA2d,
    //     m_vision.isCameraConnected(Vision.aprilTagLimelightCameraA),
    //     m_limelightA2d_originalColor);
    updateLimelightColor(
        m_limelightB2d,
        m_vision.isCameraConnected(Vision.aprilTagLimelightCameraB),
        m_limelightB2d_originalColor);
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
    if (m_vision != null) updateLimelights();
    if (m_led != null) updateLED();
  }

  @Override
  public void close() throws Exception {
    for (var display : m_displays) display.close();
    m_armVisualizer2.close();
    m_climberVisualizer2.close();
  }
}
