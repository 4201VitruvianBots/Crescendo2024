// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Controls;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise.
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall. Use the {@link #allianceFlip(Translation2d)} and {@link #allianceFlip(Pose2d)}
 * methods to flip these values based on the current alliance color.
 */
public final class SimConstants {
  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth = Units.inchesToMeters(315.5);
  public static final double tapeWidth = Units.inchesToMeters(2.0);
  public static final double aprilTagWidth = Units.inchesToMeters(6.0);
  public static final double fieldHeightMeters = Units.feetToMeters(27);

  /**
   * Flips a translation to the correct side of the field based on the current alliance color. By
   * default, all translations and poses in {@link SimConstants} are stored with the origin at the
   * rightmost point on the BLUE ALLIANCE wall.
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
   * all translations and poses in {@link SimConstants} are stored with the origin at the rightmost
   * point on the BLUE ALLIANCE wall.
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
