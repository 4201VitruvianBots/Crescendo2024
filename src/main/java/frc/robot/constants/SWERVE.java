package frc.robot.constants;

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

    public static double frontLeftCANCoderOffset = -197.75376;
    public static double frontRightCANCoderOffset = -352.61712;
    public static double backLeftCANCoderOffset = -10.1952;
    public static double backRightCANCoderOffset = -211.55256;

    public static double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
    public static final double kLimitedSpeedMetersPerSecond = kMaxSpeedMetersPerSecond / 5;
    public static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;
    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2.0;
    public static final double kLimitedRotationRadiansPerSecond = kMaxRotationRadiansPerSecond / 5;

    public static final double kP_X = 2.5;
    public static final double kI_X = 0;
    public static final double kD_X = 0;
    public static final double kP_Y = 2.5;
    public static final double kI_Y = 0;
    public static final double kD_Y = 0;

    public static double kP_Theta = 5.0;
    public static double kI_Theta = 0;
    public static double kD_Theta = 0.5;
  }

  public static class MODULE {
    public static final double kDriveMotorGearRatio = 6.12;
    public static final double kTurnMotorGearRatio = 150.0 / 7.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);
    public static final DCMotor kTurnGearbox = DCMotor.getFalcon500(1);

    public static final double kSlipCurrent = 300.0;
    public static final double kDriveInertia = 0.001;
    public static final double kTurnInertia = 0.00001;
    public static final boolean kTurnInverted = true;

    public static final double ksDriveVoltSecondsPerMeter = 0.605 / 12;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 1.72 / 12;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.193 / 12;

    public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
  }
}
