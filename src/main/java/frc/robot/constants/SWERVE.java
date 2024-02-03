package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.ModuleMap;
import frc.robot.utils.ModuleMap.MODULE_POSITION;
import java.util.Map;

public final class SWERVE {

  public static final class DRIVE {
    public static final double kTrackWidth = Units.inchesToMeters(24);
    public static final double kWheelBase = Units.inchesToMeters(24);

    public static final Map<MODULE_POSITION, Translation2d> kModuleTranslations =
        Map.of(
            MODULE_POSITION.FRONT_LEFT,
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            MODULE_POSITION.FRONT_RIGHT,
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            MODULE_POSITION.BACK_LEFT,
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            MODULE_POSITION.BACK_RIGHT,
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(
            ModuleMap.orderedValues(kModuleTranslations, new Translation2d[0]));

    //    public static double frontLeftCANCoderOffset = 197.75376;
    //    public static double frontRightCANCoderOffset = 352.61712;
    //    public static double backLeftCANCoderOffset = 10.1952;
    //    public static double backRightCANCoderOffset = 211.55256;
    // In rotations
    public static double kFrontLeftEncoderOffset = -0.047607421875;
    public static double kFrontRightEncoderOffset = -0.975830078125;
    public static double kBackLeftEncoderOffset = -0.527099609375;
    public static double kBackRightEncoderOffset = -0.587646484375;

    private static final boolean kInvertLeftDrive = true;
    private static final boolean kInvertRightDrive = false;

    public static double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
    public static final double kLimitedSpeedMetersPerSecond = kMaxSpeedMetersPerSecond / 5;
    public static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;
    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2.0;
    public static final double kLimitedRotationRadiansPerSecond = kMaxRotationRadiansPerSecond / 5;
    // Driving
    public static final double kP_X = 7;
    public static final double kI_X = 0;
    public static final double kD_X = 0;
    // Rotation
    public static double kP_Theta = 8.0;
    public static double kI_Theta = 0;
    public static double kD_Theta = 0.5;
  }

  //    public static double frontLeftCANCoderOffset = 197.75376;
  //    public static double frontRightCANCoderOffset = 352.61712;
  //    public static double backLeftCANCoderOffset = 10.1952;
  //    public static double backRightCANCoderOffset = 211.55256;
  // In rotations
  public static double kFrontLeftEncoderOffset = -0.0478515625;
  public static double kFrontRightEncoderOffset = 0.0302734375;
  public static double kBackLeftEncoderOffset = 0.473388671875;
  public static double kBackRightEncoderOffset = 0.413818359375;

  private static final boolean kInvertLeftTurn = true;
  private static final boolean kInvertRightTurn = false;

  public static double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
  public static final double kLimitedSpeedMetersPerSecond = kMaxSpeedMetersPerSecond / 5;
  public static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;
  public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2.0;
  public static final double kLimitedRotationRadiansPerSecond = kMaxRotationRadiansPerSecond / 5;

  public static final double kSlipCurrent = 300.0;
  public static final double kDriveInertia = 0.001;
  public static final double kTurnInertia = 0.00001;
  public static final boolean kTurnInverted = true;

  public static double kP_Theta = 8.0;
  public static double kI_Theta = 0;
  public static double kD_Theta = 0.5;

  public static class MODULE {
    public static final double kDriveMotorGearRatio = 6.12;
    public static final double kTurnMotorGearRatio = 150.0 / 7.0;
    public static final double kCoupleRatio = 3.5714285714285716;
    public static final double kWheelRadiusInches = 2;
    public static final double kWheelDiameterMeters = 2 * Units.inchesToMeters(kWheelRadiusInches);

    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);
    public static final DCMotor kTurnGearbox = DCMotor.getFalcon500(1);

    public static final double kSlipCurrent = 300.0;
    public static final double kFrictionVoltage = 0.25;
    public static final double kDriveInertia = 0.001;
    public static final double kTurnInertia = 0.00001;
    public static final boolean kTurnInverted = true;

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs turnGains =
        new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains =
        new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

    private static final SwerveModule.ClosedLoopOutputType turnClosedLoopOutput =
        SwerveModule.ClosedLoopOutputType.Voltage;
    private static final SwerveModule.ClosedLoopOutputType driveClosedLoopOutput =
        SwerveModule.ClosedLoopOutputType.Voltage;

    //    public static final double ksDriveVoltsRotation = 0.11286;
    //    public static final double kvDriveVoltSecondsPerRotation = 0.10079;
    //    public static final double kaDriveVoltSecondsSquaredPerRotation = 0.040151;

    public static final double ksDriveVoltsRotation = 0.24085;
    public static final double kvDriveVoltSecondsPerRotation = 2.4597;
    public static final double kaDriveVoltSecondsSquaredPerRotation = 0.033818;

    //    public static final double ksDriveVoltsRotation = (0.11286 / 12.0);
    //    public static final double kvDriveVoltSecondsPerRotation = (0.10079 / 12.0);
    //    public static final double kaDriveVoltSecondsSquaredPerRotation = (0.040151 / 12.0);

    //    public static final double ksDriveVoltsRotation = (0.32 / 12);
    //    public static final double kvDriveVoltSecondsPerRotation = (1.51 / 12);
    //    public static final double kaDriveVoltSecondsSquaredPerRotation = (0.27 / 12);

    public static final double ksTurnVoltsRotation = (0.24085 / 12.0);
    public static final double kvTurnVoltSecondsPerRotation = (2.4597 / 12.0);
    public static final double kaTurnVoltSecondsSquaredPerRotation = (0.033818 / 12.0);
  }

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants().withPigeon2Id(CAN.pigeon).withCANbusName(CAN.rioCanbus);

  private static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(MODULE.kDriveMotorGearRatio)
          .withSteerMotorGearRatio(MODULE.kTurnMotorGearRatio)
          .withWheelRadius(MODULE.kWheelRadiusInches)
          .withSlipCurrent(MODULE.kSlipCurrent)
          .withSteerMotorGains(MODULE.turnGains)
          .withDriveMotorGains(MODULE.driveGains)
          .withSteerMotorClosedLoopOutput(MODULE.turnClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(MODULE.driveClosedLoopOutput)
          .withSpeedAt12VoltsMps(DRIVE.kMaxSpeedMetersPerSecond)
          .withSteerInertia(MODULE.kTurnInertia)
          .withDriveInertia(MODULE.kDriveInertia)
          .withSteerFrictionVoltage(MODULE.kFrictionVoltage)
          .withDriveFrictionVoltage(MODULE.kFrictionVoltage)
          .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(MODULE.kCoupleRatio)
          .withSteerMotorInverted(MODULE.kTurnInverted);

  public static final SwerveModuleConstants FrontLeftConstants =
      ConstantCreator.createModuleConstants(
          CAN.frontLeftTurnMotor,
          CAN.frontLeftDriveMotor,
          CAN.frontLeftCanCoder,
          DRIVE.kFrontLeftEncoderOffset,
          DRIVE.kModuleTranslations.get(MODULE_POSITION.FRONT_LEFT).getX(),
          DRIVE.kModuleTranslations.get(MODULE_POSITION.FRONT_LEFT).getY(),
          DRIVE.kInvertLeftDrive);
  public static final SwerveModuleConstants FrontRightConstants =
      ConstantCreator.createModuleConstants(
          CAN.frontRightTurnMotor,
          CAN.frontRightDriveMotor,
          CAN.frontRightCanCoder,
          DRIVE.kFrontRightEncoderOffset,
          DRIVE.kModuleTranslations.get(MODULE_POSITION.FRONT_RIGHT).getX(),
          DRIVE.kModuleTranslations.get(MODULE_POSITION.FRONT_RIGHT).getY(),
          DRIVE.kInvertRightDrive);
  public static final SwerveModuleConstants BackLeftConstants =
      ConstantCreator.createModuleConstants(
          CAN.backLeftTurnMotor,
          CAN.backLeftDriveMotor,
          CAN.backLeftCanCoder,
          DRIVE.kBackLeftEncoderOffset,
          DRIVE.kModuleTranslations.get(MODULE_POSITION.BACK_LEFT).getX(),
          DRIVE.kModuleTranslations.get(MODULE_POSITION.BACK_LEFT).getY(),
          DRIVE.kInvertLeftDrive);
  public static final SwerveModuleConstants BackRightConstants =
      ConstantCreator.createModuleConstants(
          CAN.backRightTurnMotor,
          CAN.backRightDriveMotor,
          CAN.backRightCanCoder,
          DRIVE.kBackRightEncoderOffset,
          DRIVE.kModuleTranslations.get(MODULE_POSITION.BACK_RIGHT).getX(),
          DRIVE.kModuleTranslations.get(MODULE_POSITION.BACK_RIGHT).getY(),
          DRIVE.kInvertRightDrive);
}
