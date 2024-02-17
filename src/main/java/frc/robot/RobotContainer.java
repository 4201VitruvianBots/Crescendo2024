// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.SWERVE.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.drive.ResetGyro;
import frc.robot.commands.amp.ArmJoystickSetpoint;
import frc.robot.commands.amp.RunAmp;
import frc.robot.commands.autos.DriveStraightChoreoTest;
import frc.robot.commands.autos.DriveStraightPathPlannerTest;
import frc.robot.commands.autos.FourPieceNear;
import frc.robot.commands.autos.ThreePieceFar;
import frc.robot.commands.characterization.SwerveDriveDynamic;
import frc.robot.commands.characterization.SwerveDriveQuasistatic;
import frc.robot.commands.characterization.SwerveTurnDynamic;
import frc.robot.commands.characterization.SwerveTurnQuasistatic;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.Runfull;
import frc.robot.commands.intake.SetIntakePercentOutput;
// import frc.robot.commands.shooter.ShootNStrafe;
import frc.robot.commands.shooter.SetShooterRPMSetpoint;
import frc.robot.commands.shooter.ShootNStrafe;
import frc.robot.commands.shooter.ToggleShooterTestMode;
import frc.robot.constants.AMP;
// import frc.robot.commands.shooter.SetAndHoldPercentOutputSetpoint;
// import frc.robot.commands.uptake.RunUptake;
import frc.robot.constants.ROBOT;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.constants.USB;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.*;
import frc.robot.utils.SysIdUtils;
import frc.robot.utils.Telemetry;
import frc.robot.visualizers.SuperStructureVisualizer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final CommandSwerveDrivetrain m_swerveDrive =
      new CommandSwerveDrivetrain(
          DrivetrainConstants,
          FrontLeftConstants,
          FrontRightConstants,
          BackLeftConstants,
          BackRightConstants);
  private final Telemetry m_telemetry = new Telemetry();
  private final Vision m_vision = new Vision();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Arm m_arm = new Arm();
  private final AmpShooter m_ampShooter = new AmpShooter();
  private final Climber m_climber = new Climber();
  private final RobotTime m_robotTime = new RobotTime();
  private final Controls m_controls = new Controls();
  private final LEDSubsystem m_led = new LEDSubsystem();

  private final FieldSim m_fieldSim = new FieldSim();
  private SuperStructureVisualizer m_visualizer;

  private final LoggedDashboardChooser<Command> m_autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");
  private final LoggedDashboardChooser<Command> m_sysidChooser =
      new LoggedDashboardChooser<>("SysID Chooser");

  private final Joystick leftJoystick = new Joystick(USB.leftJoystick);
  private final Joystick rightJoystick = new Joystick(USB.rightJoystick);
  private final CommandXboxController xboxController =
      new CommandXboxController(USB.xBoxController);
  private final PS4Controller m_testController = new PS4Controller(USB.testController);

  public RobotContainer() {
    m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
    m_telemetry.registerFieldSim(m_fieldSim);
    m_controls.registerDriveTrain(m_swerveDrive);
    m_controls.registerArm(m_arm);
    initializeSubsystems();
    configureBindings();
    initAutoChooser();

    SmartDashboard.putData("ResetGyro", new ResetGyro(m_swerveDrive));

    if (ROBOT.useSysID) initSysidChooser();

    if (RobotBase.isSimulation()) {
      m_visualizer = new SuperStructureVisualizer();
      m_visualizer.registerIntake(m_intake);
      m_visualizer.registerShooter(m_shooter);
      m_visualizer.registerAmpShooter(m_ampShooter);
      m_visualizer.registerArm(m_arm);
      m_visualizer.registerClimber(m_climber);
      m_visualizer.registerVision(m_vision);
      m_visualizer.registerLedSubsystem(m_led);
    }
  }

  private void initializeSubsystems() {
    if (RobotBase.isReal()) {
      m_swerveDrive.setDefaultCommand(
          m_swerveDrive.applyFieldCentricDrive(
              () ->
                  new ChassisSpeeds(
                      leftJoystick.getRawAxis(1) * DRIVE.kMaxSpeedMetersPerSecond,
                      leftJoystick.getRawAxis(0) * DRIVE.kMaxSpeedMetersPerSecond,
                      rightJoystick.getRawAxis(0) * DRIVE.kMaxRotationRadiansPerSecond)));
      //      m_swerveDrive.setDefaultCommand(
      //          m_swerveDrive.applyRequest(
      //              () ->
      //                  drive
      //                      .withVelocityX(
      //                          leftJoystick.getRawAxis(1)
      //                              * DRIVE.kMaxSpeedMetersPerSecond) // Drive forward with
      //                      // negative Y (forward)
      //                      .withVelocityY(
      //                          leftJoystick.getRawAxis(0)
      //                              * DRIVE.kMaxSpeedMetersPerSecond) // Drive left with negative
      // X (left)
      //                      .withRotationalRate(
      //                          rightJoystick.getRawAxis(0)
      //                              * DRIVE
      //                                  .kMaxRotationRadiansPerSecond))); // Drive
      // counterclockwise with
      //      // negative X (left)
    } else {
      m_swerveDrive.setDefaultCommand(
          m_swerveDrive.applyFieldCentricDrive(
              () ->
                  new ChassisSpeeds(
                      -m_testController.getRawAxis(1) * DRIVE.kMaxSpeedMetersPerSecond,
                      -m_testController.getRawAxis(0) * DRIVE.kMaxSpeedMetersPerSecond,
                      -m_testController.getRawAxis(2) * DRIVE.kMaxRotationRadiansPerSecond)));
      //      m_swerveDrive.setDefaultCommand(
      //          m_swerveDrive.applyRequest(
      //              () ->
      //                  drive
      //                      .withVelocityX(
      //                          -m_testController.getRawAxis(1)
      //                              * DRIVE.kMaxSpeedMetersPerSecond) // Drive forward with
      //                      // negative Y (forward)
      //                      .withVelocityY(
      //                          -m_testController.getRawAxis(0)
      //                              * DRIVE.kMaxSpeedMetersPerSecond) // Drive left with negative
      // X (left)
      //                      .withRotationalRate(
      //                          -m_testController.getRawAxis(2)
      //                              * DRIVE
      //                                  .kMaxRotationRadiansPerSecond))); // Drive
      // counterclockwise with
      // negative X (left)
    }

    m_intake.setDefaultCommand(
        new SetIntakePercentOutput(
            m_intake, xboxController.getLeftY(), xboxController.getRightY()));
    m_arm.setDefaultCommand(new ArmJoystickSetpoint(m_arm, () -> -xboxController.getLeftY()));
  }

  private void configureBindings() {
    xboxController
        .a()
        .whileTrue(new SetShooterRPMSetpoint(m_shooter, RPM_SETPOINT.SLOW.get())); // slow sbeaker
    xboxController
        .b()
        .whileTrue(
            new SetShooterRPMSetpoint(m_shooter, RPM_SETPOINT.SPEAKER.get())); // fast sbeaker
    xboxController.rightTrigger().whileTrue(new RunIntake(m_intake, -0.5, -0.5));

    xboxController.rightBumper().whileTrue(new Runfull(m_intake, -0.55, -0.85, m_ampShooter, 0.5));
    xboxController.leftBumper().whileTrue(new Runfull(m_intake, 0.50, 0.85, m_ampShooter, -0.5));
    //    xboxController.povDown().whileTrue(new RunUptake(m_uptake, -0.5));
    //    xboxController.povUp().whileTrue(new RunUptake(m_uptake, 0.5));
    xboxController
        .y()
        .whileTrue(
            new ShootNStrafe(
                m_swerveDrive,
                m_ampShooter,
                m_shooter,
                () -> -leftJoystick.getRawAxis(1) * DRIVE.kMaxSpeedMetersPerSecond,
                () -> -leftJoystick.getRawAxis(1) * DRIVE.kMaxSpeedMetersPerSecond,
                () -> -rightJoystick.getRawAxis(1) * DRIVE.kMaxSpeedMetersPerSecond,
                4201));

    xboxController
        .povDown()
        .whileTrue(new RunAmp(m_ampShooter, AMP.INTAKE_STATE.REVERSE_SLOW.get()));
  }

  public void initAutoChooser() {
    m_autoChooser.addDefaultOption("Do Nothing", new WaitCommand(0));
    m_autoChooser.addOption(
        "DriveStraightPathPlannerTest",
        new DriveStraightPathPlannerTest(m_swerveDrive, m_fieldSim));
    m_autoChooser.addOption("FourPieceNear", new FourPieceNear(m_swerveDrive, m_fieldSim));
    m_autoChooser.addOption("ThreePieceFar", new ThreePieceFar(m_swerveDrive, m_fieldSim));
    m_autoChooser.addOption(
        "DriveStraightChoreoTest", new DriveStraightChoreoTest(m_swerveDrive, m_fieldSim));
  }

  public void initSysidChooser() {
    SysIdUtils.createSwerveDriveRoutines(m_swerveDrive);
    SysIdUtils.createSwerveTurnRoutines(m_swerveDrive);

    SmartDashboard.putData("toggleShooterTestMode", new ToggleShooterTestMode(m_shooter));

    SmartDashboard.putData(
        "Start Logging", new InstantCommand(SignalLogger::start).ignoringDisable(true));
    SmartDashboard.putData(
        "Stop Logging", new InstantCommand(SignalLogger::stop).ignoringDisable(true));
    SmartDashboard.putData(
        "initDriveSettings",
        new InstantCommand(m_swerveDrive::initDriveSysid).ignoringDisable(true));
    SmartDashboard.putData(
        "initTurnSettings", new InstantCommand(m_swerveDrive::initTurnSysid).ignoringDisable(true));

    m_sysidChooser.addOption(
        "driveQuasistaticForward",
        new SwerveDriveQuasistatic(m_swerveDrive, SysIdRoutine.Direction.kForward));
    m_sysidChooser.addOption(
        "driveQuasistaticBackwards",
        new SwerveDriveQuasistatic(m_swerveDrive, SysIdRoutine.Direction.kReverse));
    m_sysidChooser.addOption(
        "driveDynamicForward",
        new SwerveDriveDynamic(m_swerveDrive, SysIdRoutine.Direction.kForward));
    m_sysidChooser.addOption(
        "driveDynamicBackward",
        new SwerveDriveDynamic(m_swerveDrive, SysIdRoutine.Direction.kReverse));

    m_sysidChooser.addOption(
        "turnQuasistaticForward",
        new SwerveTurnQuasistatic(m_swerveDrive, SysIdRoutine.Direction.kForward));
    m_sysidChooser.addOption(
        "turnQuasistaticBackwards",
        new SwerveTurnQuasistatic(m_swerveDrive, SysIdRoutine.Direction.kReverse));
    m_sysidChooser.addOption(
        "turnDynamicForward",
        new SwerveTurnDynamic(m_swerveDrive, SysIdRoutine.Direction.kForward));
    m_sysidChooser.addOption(
        "turnDynamicBackward",
        new SwerveTurnDynamic(m_swerveDrive, SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    if (ROBOT.useSysID) return m_sysidChooser.get();
    else return m_autoChooser.get();
  }

  public void periodic() {
    if (DriverStation.isDisabled()) {
      m_controls.updateStartPose(m_autoChooser.getSendableChooser().getSelected());
    }

    if (m_visualizer != null) m_visualizer.periodic();
  }

  public void testInit() {
    m_arm.testInit();
  }

  public void testPeriodic() {
    m_arm.testPeriodic();
  }
}
