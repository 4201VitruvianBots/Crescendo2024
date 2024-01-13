package frc.robot.simulation;

import static frc.robot.constants.SIM.fieldLength;
import static frc.robot.constants.SIM.fieldWidth;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.SIM;
import frc.robot.subsystems.Controls;

public class SimUtils {
  /**
   * Flips a translation to the correct side of the field based on the current alliance color. By
   * default, all translations and poses in {@link SIM} are stored with the origin at the rightmost
   * point on the BLUE ALLIANCE wall.
   */
  public static Translation2d allianceFlip(Translation2d translation) {
    if (Controls.getAllianceColor() == DriverStation.Alliance.Red) {
      return new Translation2d(fieldLength - translation.getX(), translation.getY());
    } else {
      return translation;
    }
  }

  public static Pose2d absoluteFlip(Pose2d pose) {
    return new Pose2d(
        absoluteFlip(pose.getTranslation()),
        new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
  }

  public static Translation2d absoluteFlip(Translation2d translation) {
    return new Translation2d(fieldLength - translation.getX(), translation.getY());
  }

  /**
   * Flips a pose to the correct side of the field based on the current alliance color. By default,
   * all translations and poses in {@link SIM} are stored with the origin at the rightmost point on
   * the BLUE ALLIANCE wall.
   */
  public static Pose2d allianceFlip(Pose2d pose) {
    if (Controls.getAllianceColor() == DriverStation.Alliance.Red) {
      return new Pose2d(
          fieldLength - pose.getX(),
          pose.getY(),
          new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }

  public static Pose2d pathPlannerFlip(Pose2d pose) {
    if (Controls.getAllianceColor() == DriverStation.Alliance.Red) {
      return new Pose2d(pose.getX(), fieldWidth - pose.getY(), pose.getRotation());
    } else {
      return pose;
    }
  }
}
