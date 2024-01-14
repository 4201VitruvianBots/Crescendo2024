// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
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
import frc.robot.commands.autos.DriveStriaghtTest;
import frc.robot.commands.characterization.SwerveDriveDynamic;
import frc.robot.commands.characterization.SwerveDriveQuasistatic;
import frc.robot.commands.characterization.SwerveTurnDynamic;
import frc.robot.commands.characterization.SwerveTurnQuasistatic;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.constants.BASE;
import frc.robot.constants.USB;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.RobotTime;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.ModuleMap;
import frc.robot.utils.SysidUtils;

public class RobotContainer {
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final Controls m_controls = new Controls();
  private final FieldSim m_fieldSim = new FieldSim(m_swerveDrive);
  private final RobotTime m_robotTime = new RobotTime();

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final SendableChooser<Command> m_sysidChooser = new SendableChooser<>();

  private final Joystick leftJoystick = new Joystick(USB.leftJoystick);
  private final Joystick rightJoystick = new Joystick(USB.rightJoystick);
  private final CommandXboxController xboxController =
      new CommandXboxController(USB.xBoxController);
  private final PS4Controller m_testController = new PS4Controller(USB.testController);

  public RobotContainer() {
    initializeSubsystems();
    configureBindings();
    initAutoChooser();
    initSysidChooser();
  }

  private void initializeSubsystems() {
    if (RobotBase.isReal()) {
      m_swerveDrive.setDefaultCommand(
          new SetSwerveDrive(
              m_swerveDrive,
              () -> leftJoystick.getRawAxis(1),
              () -> leftJoystick.getRawAxis(0),
              () -> rightJoystick.getRawAxis(0)));
    } else {
      m_swerveDrive.setDefaultCommand(
          new SetSwerveDrive(
              m_swerveDrive,
              () -> -m_testController.getRawAxis(1),
              () -> -m_testController.getRawAxis(0),
              () -> -m_testController.getRawAxis(2)));
    }
  }

  private void configureBindings() {}

  public void initAutoChooser() {
    m_autoChooser.addOption("do nothing", new DriveStriaghtTest(m_swerveDrive));
    // m_autoChooser.addOption("Minimalauto1", new Minimalauto1(m_swerveDrive));
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    // m_autoChooser.addOption("Minimalauto2", new Minimalauto2(m_swerveDrive));
    // m_autoChooser.addOption("Minimalauto3", new Minimalauto3(m_swerveDrive));
    // m_autoChooser.addOption("DefAuto", new DefAuto(m_swerveDrive));
    SmartDashboard.putData("AutoChooser", m_autoChooser);
  }

  public void initSysidChooser() {
    SysidUtils.createSwerveDriveRoutines(m_swerveDrive);
    SysidUtils.createSwerveTurnRoutines(m_swerveDrive);

    SmartDashboard.putData("Start Logging", new InstantCommand(SignalLogger::start));
    SmartDashboard.putData("Stop Logging", new InstantCommand(SignalLogger::stop));

    m_sysidChooser.addOption("initDriveSettings", new InstantCommand(m_swerveDrive::initDriveSysid));
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

    m_sysidChooser.addOption("initTurnSettings", new InstantCommand(()-> m_swerveDrive.getSwerveModule(ModuleMap.MODULE_POSITION.FRONT_LEFT).initTurnSysid());
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
    if (BASE.useSysID) return m_sysidChooser.getSelected();
    else return m_autoChooser.getSelected();
  }
}
