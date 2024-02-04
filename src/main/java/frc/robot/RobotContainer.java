// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.SWERVE.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.amp.ArmForward;
import frc.robot.commands.autos.DriveStraightChoreoTest;
import frc.robot.commands.characterization.SwerveDriveDynamic;
import frc.robot.commands.characterization.SwerveDriveQuasistatic;
import frc.robot.commands.characterization.SwerveTurnDynamic;
import frc.robot.commands.characterization.SwerveTurnQuasistatic;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.SetIntakePercentOutput;
import frc.robot.commands.shooter.SetAndHoldRPMSetpoint;
import frc.robot.commands.uptake.RunUptake;
import frc.robot.constants.ROBOT;
import frc.robot.constants.SWERVE;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.constants.USB;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.*;
import frc.robot.utils.SysidUtils;
import frc.robot.utils.Telemetry;
import frc.robot.visualizers.SuperStructureVisualizer;

public class RobotContainer {
  //  private final SwerveDrive m_swerveDrive = new SwerveDrive();
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
  private final Uptake m_uptake = new Uptake();
  private final Shooter m_shooter = new Shooter();
  private final Arm m_flipper = new Arm();
  private final AmpShooter m_ampShooter = new AmpShooter();
  private final Climber m_climber = new Climber();
  private final RobotTime m_robotTime = new RobotTime();
  private final Controls m_controls = new Controls();
  private final LEDSubsystem m_led = new LEDSubsystem(m_controls);
  private final FieldSim m_fieldSim = new FieldSim();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
          .withRotationalDeadband(
              SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop

  private final SuperStructureVisualizer m_visualizer =
      new SuperStructureVisualizer(
          m_intake, m_uptake, m_shooter, m_ampShooter, m_flipper, m_climber, m_vision);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final SendableChooser<Command> m_sysidChooser = new SendableChooser<>();

  private final Joystick leftJoystick = new Joystick(USB.leftJoystick);
  private final Joystick rightJoystick = new Joystick(USB.rightJoystick);
  private final CommandXboxController xboxController =
      new CommandXboxController(USB.xBoxController);
  private final PS4Controller m_testController = new PS4Controller(USB.testController);

  public RobotContainer() {
    m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
    m_telemetry.registerFieldSim(m_fieldSim);
    m_vision.registerFieldSim(m_fieldSim);
    initializeSubsystems();
    configureBindings();
    initAutoChooser();

    if (ROBOT.useSysID) initSysidChooser();
  }

  private void initializeSubsystems() {
    if (RobotBase.isReal()) {
      m_swerveDrive.setDefaultCommand(
          m_swerveDrive.applyRequest(
              () ->
                  drive
                      .withVelocityX(
                          leftJoystick.getRawAxis(1)
                              * DRIVE.kMaxSpeedMetersPerSecond) // Drive forward with
                      // negative Y (forward)
                      .withVelocityY(
                          leftJoystick.getRawAxis(0)
                              * DRIVE.kMaxSpeedMetersPerSecond) // Drive left with negative X (left)
                      .withRotationalRate(
                          rightJoystick.getRawAxis(0)
                              * DRIVE
                                  .kMaxRotationRadiansPerSecond))); // Drive counterclockwise with
      // negative X (left)
    } else {
      m_swerveDrive.setDefaultCommand(
          m_swerveDrive.applyRequest(
              () ->
                  drive
                      .withVelocityX(
                          -m_testController.getRawAxis(1)
                              * DRIVE.kMaxSpeedMetersPerSecond) // Drive forward with
                      // negative Y (forward)
                      .withVelocityY(
                          -m_testController.getRawAxis(0)
                              * DRIVE.kMaxSpeedMetersPerSecond) // Drive left with negative X (left)
                      .withRotationalRate(
                          -m_testController.getRawAxis(2)
                              * DRIVE
                                  .kMaxRotationRadiansPerSecond))); // Drive counterclockwise with
      // negative X (left)
    }

    m_intake.setDefaultCommand(
        new SetIntakePercentOutput(
            m_intake, xboxController.getLeftY(), xboxController.getRightY()));
  }

  private void configureBindings() {
    //    xboxController.b().whileTrue(new SetIntakePercentOutput(m_intake, -0.85, -0.85));
    //    xboxController.a().whileTrue(new SetIntakePercentOutput(m_intake, -0.75, -0.75));
    //    xboxController.y().whileTrue(new SetIntakePercentOutput(m_intake, -1.0, -1.0));

    xboxController.a().whileTrue(new SetAndHoldRPMSetpoint(m_shooter, 420.69)); // amp
    xboxController.b().whileTrue(new SetAndHoldRPMSetpoint(m_shooter, 420.69)); // sbeaker
    xboxController.rightBumper().whileTrue(new RunIntake(m_intake, 0.5));
    xboxController.povDown().whileTrue(new RunUptake(m_uptake, -0.5));
    xboxController.povUp().whileTrue(new RunUptake(m_uptake, 0.5));
    xboxController.y().whileTrue(new ArmForward(m_flipper));
  }

  public void initAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    m_autoChooser.addOption(
        "DriveStraightChoreoTest", new DriveStraightChoreoTest(m_swerveDrive, m_fieldSim));
    m_autoChooser.addOption(
        "DriveStraightChoreoTest", new DriveStraightChoreoTest(m_swerveDrive, m_fieldSim));
    // m_autoChooser.addOption("Minimalauto1", new Minimalauto1(m_swerveDrive));
    // m_autoChooser.addOption("Minimalauto2", new Minimalauto2(m_swerveDrive));
    // m_autoChooser.addOption("Minimalauto3", new Minimalauto3(m_swerveDrive));
    // m_autoChooser.addOption("DefAuto", new DefAuto(m_swerveDrive));
    //    m_autoChooser.addOption("Amp Test", new ScoreAmp(m_flipper, m_ampshooter));
    //    m_autoChooser.addOption("Speaker Test", new ScoreSpeaker(m_shooter, m_uptake));
    SmartDashboard.putData("AutoChooser", m_autoChooser);
  }

  public void initSysidChooser() {
    SysidUtils.createSwerveDriveRoutines(m_swerveDrive);
    SysidUtils.createSwerveTurnRoutines(m_swerveDrive);

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

    SmartDashboard.putData("SysID Chooser", m_sysidChooser);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    if (ROBOT.useSysID) return m_sysidChooser.getSelected();
    else return m_autoChooser.getSelected();
  }

  public void periodic() {
    final var globalPose = m_vision.getEstimatedGlobalPose();
    if (globalPose.isPresent()) {
      m_swerveDrive.addVisionMeasurement(
          globalPose.get().estimatedPose.toPose2d(), globalPose.get().timestampSeconds);
    }
    m_fieldSim.periodic();
    m_visualizer.periodic();
  }
}
