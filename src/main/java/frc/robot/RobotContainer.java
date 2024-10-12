// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ampShooter.RunAmpSensored;
import frc.robot.commands.arm.ArmJoystick;
import frc.robot.commands.arm.ArmSetpoint;
import frc.robot.commands.arm.ToggleArmControlMode;
import frc.robot.commands.autos.*;
import frc.robot.commands.characterization.SwerveDriveDynamic;
import frc.robot.commands.characterization.SwerveDriveQuasistatic;
import frc.robot.commands.characterization.SwerveTurnDynamic;
import frc.robot.commands.characterization.SwerveTurnQuasistatic;
import frc.robot.commands.climber.ResetClimberHeight;
import frc.robot.commands.climber.ToggleClimbMode;
import frc.robot.commands.drive.ResetGyro;
import frc.robot.commands.drive.SetTrackingState;
import frc.robot.commands.intake.AmpIntake;
import frc.robot.commands.intake.AmpOuttake;
import frc.robot.commands.intake.AutoRunAmpTakeTwo;
import frc.robot.commands.led.GetSubsystemStates;
import frc.robot.commands.shooter.AutoSetRPMSetpoint;
import frc.robot.commands.shooter.RunKicker;
import frc.robot.commands.shooter.SetShooterRPMSetpoint;
import frc.robot.constants.*;
import frc.robot.constants.AMPSHOOTER.STATE;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.constants.VISION.TRACKING_STATE;
import frc.robot.subsystems.*;
import frc.robot.utils.SysIdArmUtils;
import frc.robot.utils.SysIdShooterUtils;
import frc.robot.utils.SysIdUtils;
import frc.robot.visualizers.SuperStructureVisualizer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final CommandSwerveDrivetrain m_swerveDrive =
      new CommandSwerveDrivetrain(
          SWERVE.DrivetrainConstants,
          SWERVE.FrontLeftConstants,
          SWERVE.FrontRightConstants,
          SWERVE.BackLeftConstants,
          SWERVE.BackRightConstants);
//   private final Telemetry m_telemetry = new Telemetry();
//   private final Vision m_vision = new Vision();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Arm m_arm = new Arm();
  private final AmpShooter m_ampShooter = new AmpShooter();
//   private final Climber m_climber = new Climber();
//   private final RobotTime m_robotTime = new RobotTime();
  private final Controls m_controls = new Controls();
  private final LEDSubsystem m_led = new LEDSubsystem();

//   private final FieldSim m_fieldSim = new FieldSim();
  private SuperStructureVisualizer m_visualizer;

  private final LoggedDashboardChooser<Command> m_autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");
  private final LoggedDashboardChooser<Command> m_sysidChooser =
      new LoggedDashboardChooser<>("SysID Chooser");

  //   private final Joystick leftJoystick = new Joystick(USB.leftJoystick);
  //   private final Joystick rightJoystick = new Joystick(USB.rightJoystick);
  private final CommandXboxController driveController =
      new CommandXboxController(USB.driveController);
  private final CommandXboxController xboxController =
      new CommandXboxController(USB.xBoxController);
  private final CommandPS4Controller m_testController =
      new CommandPS4Controller(USB.testController);
//   private final Trigger trigger =
//       new Trigger(xboxController.leftStick().and(xboxController.rightStick()));

  public RobotContainer() {
    // m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
    // m_swerveDrive.registerVisionSubsystem(m_vision);
    m_ampShooter.registerIntake(m_intake);
    // m_vision.registerSwerveDrive(m_swerveDrive);
    m_controls.registerDriveTrain(m_swerveDrive);
    m_controls.registerIntake(m_intake);
    m_controls.registerArm(m_arm);
    // m_controls.registerVision(m_vision);
    initializeSubsystems();
    configureBindings();
    initSmartDashboard();

    if (RobotBase.isSimulation()) {
      // m_telemetry.registerFieldSim(m_fieldSim);
      // m_vision.registerFieldSim(m_fieldSim);

      m_visualizer = new SuperStructureVisualizer();
      m_visualizer.registerIntake(m_intake);
      m_visualizer.registerShooter(m_shooter);
      m_visualizer.registerAmpShooter(m_ampShooter);
      m_visualizer.registerArm(m_arm);
      // m_visualizer.registerClimber(m_climber);
      // m_visualizer.registerVision(m_vision);
      m_visualizer.registerLedSubsystem(m_led);
    }
  }

  private void initializeSubsystems() {
    if (RobotBase.isReal()) {
      m_swerveDrive.setDefaultCommand(
          m_swerveDrive.applyChassisSpeeds(
              () ->
                  new ChassisSpeeds(
                      driveController.getLeftY() * DRIVE.kMaxSpeedMetersPerSecond,
                      driveController.getLeftX() * DRIVE.kMaxSpeedMetersPerSecond,
                      driveController.getRightX() * DRIVE.kMaxRotationRadiansPerSecond)));
    } else {
      m_swerveDrive.setDefaultCommand(
          m_swerveDrive.applyChassisSpeeds(
              () ->
                  new ChassisSpeeds(
                      -m_testController.getRawAxis(0) * DRIVE.kMaxSpeedMetersPerSecond,
                      m_testController.getRawAxis(1) * DRIVE.kMaxSpeedMetersPerSecond,
                      -m_testController.getRawAxis(2) * DRIVE.kMaxRotationRadiansPerSecond)));

      m_testController
          .cross()
          .whileTrue(new SetTrackingState(m_swerveDrive, TRACKING_STATE.SPEAKER));
    }

    // Default command to decelerate the flywheel if no other command is set
    //    m_shooter.setDefaultCommand(new DefaultFlywheel(m_shooter));
    m_arm.setDefaultCommand(new ArmJoystick(m_arm, () -> -xboxController.getLeftY()));
    // m_climber.setDefaultCommand(
    //     new RunClimberJoystick(null, () -> -xboxController.getRightY(), xboxController));
    m_led.setDefaultCommand(
        new GetSubsystemStates(m_led, m_intake, null, m_shooter, null, m_swerveDrive));
  }

  private void configureBindings() {
    // var driveShootButton = new Trigger(() -> leftJoystick.getRawButton(1));
    // driveShootButton.whileTrue(new SetTrackingState(m_swerveDrive, TRACKING_STATE.SPEAKER));

    // var driveAdjustButtonBack = new Trigger(() -> leftJoystick.getRawButton(2));
    // driveAdjustButtonBack.whileTrue(new RunAmp(m_ampShooter, 0.05));

    // var targetSpeakerButton = new Trigger(() -> rightJoystick.getRawButton(1));
    // targetSpeakerButton.whileTrue(new SetTrackingState(m_swerveDrive, TRACKING_STATE.SPEAKER));

    // var driveAdjustButtonFront = new Trigger(() -> rightJoystick.getRawButton(2));
    // driveAdjustButtonFront.whileTrue(new RunAmp(m_ampShooter, -0.05));

    // var targetNoteButton = new Trigger(() -> rightJoystick.getRawButton(2));
    // targetNoteButton.whileTrue(new SetTrackingState(m_swerveDrive, TRACKING_STATE.NOTE));

    //    var SASButton = new Trigger(() -> rightJoystick.getRawButton(2));
    //    SASButton.whileTrue(
    //        new AutoShootNStrafe(
    //            m_swerveDrive,
    //            m_telemetry,
    //            m_ampShooter,
    //            m_shooter,
    //            m_intake,
    //            () -> leftJoystick.getRawAxis(1),
    //            () -> leftJoystick.getRawAxis(0),
    //            () -> rightJoystick.getRawAxis(0),
    //            0,
    //            INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
    //            STATE.INTAKING.get(),
    //            RPM_SETPOINT.MAX.get()));

    driveController
        .rightTrigger()
        .whileTrue(new SetTrackingState(m_swerveDrive, TRACKING_STATE.SPEAKER));

    xboxController
        .b()
        .whileTrue(
            new SetShooterRPMSetpoint(
                m_shooter,
                xboxController,
                RPM_SETPOINT.MAX.get(),
                RPM_SETPOINT.MAX.get())); // fast speaker

    xboxController.a().whileTrue(new ArmSetpoint(m_arm, ARM.ARM_SETPOINT.FORWARD));
    xboxController
        .x()
        .whileTrue(
            new SetShooterRPMSetpoint(
                m_shooter,
                xboxController,
                RPM_SETPOINT.SPEAKERBOTTOM.get(),
                RPM_SETPOINT.SPEAKERBOTTOM.get()));

    // toggles the climb sequence when presses and cuts the command when pressed again
    //    trigger.onTrue(new ClimbFinal(m_ampShooter, m_swerveDrive, m_arm, m_climber));
    //xboxController.back().onTrue(new ToggleClimbMode(null, m_arm)); // Left Button

    // switch between open loop and close loop
    xboxController.start().onTrue(new ToggleArmControlMode(m_arm)); // Right Button

    xboxController
        .rightTrigger()
        .whileTrue(
            new RunKicker(
                m_intake,
                m_shooter,
                0.55,
                0.75,
                m_ampShooter,
                AMPSHOOTER.STATE.SHOOTING.get())); // Intake Note with Intake And Amp
    xboxController
        .leftTrigger()
        .whileTrue(
            new AmpIntake(
                m_intake, 0.55, 0.80, m_ampShooter, 0.4)); // Outtake Note with Intake And Amp

    xboxController
        .leftBumper()
        .whileTrue(
            new AmpOuttake(
                m_intake,
                INTAKE.STATE.FRONT_ROLLER_REVERSE.get(),
                INTAKE.STATE.BACK_ROLLER_REVERSE.get(),
                m_ampShooter,
                AMPSHOOTER.STATE.REVERSE.get())); // Intake Note with Only Intake
    xboxController
        .rightBumper()
        .whileTrue(
            new RunAmpSensored(
                m_ampShooter,
                m_intake,
                AMPSHOOTER.STATE.REVERSE_SLOW.get())); // Intake Note with Only Intake

    xboxController
        .povLeft()
        .whileTrue(
            new SetShooterRPMSetpoint(
                m_shooter, xboxController, RPM_SETPOINT.REVERSE.get(), RPM_SETPOINT.REVERSE.get()));

    xboxController
        .povDown()
        .whileTrue(
            new AmpIntake(
                m_intake,
                INTAKE.STATE.FRONT_SLOW_REVERSE.get(),
                INTAKE.STATE.BACK_SLOW_REVERSE.get(),
                m_ampShooter,
                STATE.REVERSE_SLOW.get())); // Intake Note with Only Amp

    xboxController
        .povUp()
        .whileTrue(
            new AmpIntake(
                m_intake,
                INTAKE.STATE.FRONT_SLOW_INTAKING.get(),
                INTAKE.STATE.BACK_SLOW_INTAKING.get(),
                m_ampShooter,
                AMPSHOOTER.STATE.INTAKING_SLOW.get())); // Intake Note with Only Amp
  }

  private void initSmartDashboard() {
    if (ROBOT.useSysID) initSysidChooser();
    else initAutoChooser();

    SmartDashboard.putData("ResetGyro", new ResetGyro(m_swerveDrive));
    //SmartDashboard.putData("ResetClimberHeight", new ResetClimberHeight(null, 0));
    SmartDashboard.putData(
        "ResetSetupCheck", new InstantCommand(m_controls::resetInitState).ignoringDisable(true));

    //    SmartDashboard.putData("toggleShooterTestMode", new ToggleShooterTestMode(m_shooter));
  }

  private void initAutoChooser() {
    m_autoChooser.addDefaultOption("Do Nothing", new WaitCommand(0));
    m_autoChooser.addOption(
        "OneWaitAuto",
        new OneWaitAuto(m_swerveDrive, null, m_intake, m_ampShooter, m_shooter));
    m_autoChooser.addOption(
        "FourPieceNear",
        new FourPieceNear(m_swerveDrive, m_shooter, m_ampShooter, m_intake, null));
    m_autoChooser.addOption(
        "FivePiece", new FivePiece(m_swerveDrive, null, m_intake, m_ampShooter, m_shooter));
    // m_autoChooser.addOption("ThreePieceFar", new ThreePieceFar(m_swerveDrive, m_fieldSim));
    m_autoChooser.addOption(
        "TwoPieceAuto", new TwoPiece(m_swerveDrive, null, m_intake, m_ampShooter, m_shooter));
    m_autoChooser.addOption(
        "TwoPieceFar",
        new TwoPieceFar(m_swerveDrive, null, m_intake, m_ampShooter, m_shooter));

    // Test autos
    m_autoChooser.addOption("DriveTest", new DriveStraight(m_swerveDrive, null));
    m_autoChooser.addOption(
        "IntakeTestVision",
        new IntakeTestVision(m_swerveDrive, null, m_intake, m_ampShooter, m_shooter));
    m_autoChooser.addOption(
        "TestAutoShoot",
        new AutoSetRPMSetpoint(m_shooter, SHOOTER.RPM_SETPOINT.AUTO_RPM.get())
            .andThen(
                new AutoRunAmpTakeTwo(
                    m_intake,
                    m_ampShooter,
                    INTAKE.STATE.FRONT_ROLLER_INTAKING.get(),
                    INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
                    AMPSHOOTER.STATE.INTAKING.get(),
                    m_shooter)));
    // m_autoChooser.addOption(
    //     "AutoScoreTest",
    //     new AutoScore(6
    //         m_shooter,
    //         m_ampShooter,
    //         m_intake,
    //         AMP.STATE.INTAKING.get(),
    //         RPM_SETPOINT.SPEAKER.get(),
    //         INTAKE.STATE.FRONT_ROLLER_INTAKING.get(),
    //         INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
    //         WAIT.SHOOTING.get(),
    //         5));

    //    m_autoChooser.addOption(
    //        "ScoreSpeakerTesting", new ScoreSpeaker(m_shooter, m_ampShooter, m_intake));
  }

  private void initSysidChooser() {
    SignalLogger.setPath("/media/sda1/");
    var shooterSysId = SysIdShooterUtils.createShooterRoutines(m_shooter);
    var armSysId = SysIdArmUtils.createArmRoutines(m_arm);

    SysIdUtils.createSwerveDriveRoutines(m_swerveDrive);
    SysIdUtils.createSwerveTurnRoutines(m_swerveDrive);

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

    m_sysidChooser.addOption(
        "ShooterDynamicForward", shooterSysId.dynamic(SysIdRoutine.Direction.kForward));

    m_sysidChooser.addOption(
        "ShooterDynamicReverse", shooterSysId.dynamic(SysIdRoutine.Direction.kReverse));

    m_sysidChooser.addOption(
        "ShooterQuasistaticForward", shooterSysId.quasistatic(SysIdRoutine.Direction.kForward));

    m_sysidChooser.addOption(
        "ShooterQuasistaticReverse", shooterSysId.quasistatic(SysIdRoutine.Direction.kReverse));

    m_sysidChooser.addOption(
        "ArmDynamicForward", armSysId.dynamic(SysIdRoutine.Direction.kForward));

    m_sysidChooser.addOption(
        "ArmDynamicReverse", armSysId.dynamic(SysIdRoutine.Direction.kReverse));

    m_sysidChooser.addOption(
        "ArmQuasistaticForward", armSysId.quasistatic(SysIdRoutine.Direction.kForward));

    m_sysidChooser.addOption(
        "ArmQuasistaticReverse", armSysId.quasistatic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    if (ROBOT.useSysID) return m_sysidChooser.get();
    else return m_autoChooser.get();
  }

  public void periodic() {
    try {
      if (DriverStation.isDisabled()) {
        m_controls.updateStartPose(m_autoChooser.getSendableChooser().getSelected());
      }
    } catch (Exception e) {
      System.out.println("Got the following Error:");
      e.printStackTrace();
    }

    if (m_visualizer != null) m_visualizer.periodic();
  }

  public void testInit() {
    m_arm.testInit();
  }

  public void testPeriodic() {
    m_arm.testPeriodic();
  }

  public void teleopInit() {
    m_arm.teleopInit();
   // m_climber.teleopInit();
  }

  public void autonomousInit() {
    m_arm.autonomousInit();
  }

  public void disabledInit() {
    m_swerveDrive.applyRequest(SwerveRequest.ApplyChassisSpeeds::new);
    m_swerveDrive.setTrackingState(TRACKING_STATE.NONE);
    m_intake.setSpeed(0, 0);
    m_ampShooter.setPercentOutput(0);
    m_shooter.setRPMOutput(0);
    m_shooter.setPercentOutput(0);
    m_arm.resetMotionMagicState();
  }
}
