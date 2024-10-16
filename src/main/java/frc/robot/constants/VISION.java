// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VISION {
  public static final double kLimelightHFOV = 63.12;
  public static final double kLimelightVFOV = 49.54;
  public static final double kLimelightDFOV = 75.07;

  public static final double aprilTagLimelightCameraADistanceFromCenterX =
      Units.inchesToMeters(-7.590951);
  public static final double aprilTagLimelightCameraADistanceFromCenterY =
      Units.inchesToMeters(-9.541729);
  public static final double aprilTagLimelightCameraADistanceFromGroundZ =
      Units.inchesToMeters(15.184963);
  public static final double aprilTagLimelightCameraAOffsetInRadiansRoll =
      Units.degreesToRadians(0);
  public static final double aprilTagLimelightCameraAOffsetInRadiansPitch =
      Units.degreesToRadians(6.172091);
  public static final double aprilTagLimelightCameraAOffsetInRadiansYaw =
      Units.degreesToRadians(-145.513393);

  public static final double aprilTagLimelightCameraBDistanceFromCenterX =
      Units.inchesToMeters(-6.875);
  public static final double aprilTagLimelightCameraBDistanceFromCenterY =
      Units.inchesToMeters(-1.5);
  public static final double aprilTagLimelightCameraBDistanceFromGroundZ =
      Units.inchesToMeters(21.875);
  public static final double aprilTagLimelightCameraBOffsetInRadiansRoll =
      Units.degreesToRadians(0);
  public static final double aprilTagLimelightCameraBOffsetInRadiansPitch =
      Units.degreesToRadians(-35);
  public static final double aprilTagLimelightCameraBOffsetInRadiansYaw =
      Units.degreesToRadians(180);

  public static final double noteDetectionLimelightCameraDistanceFromCenterX =
      Units.inchesToMeters(12.125);
  public static final double noteDetectionLimelightCameraDistanceFromCenterY =
      Units.inchesToMeters(0);
  public static final double noteDetectionLimelightCameraDistanceFromGroundZ =
      Units.inchesToMeters(7.040388);
  public static final double noteDetectionLimelightCameraOffsetInDegreesRoll =
      Units.degreesToRadians(0);
  public static final double noteDetectionLimelightCameraOffsetInDegreesPitch =
      Units.degreesToRadians(0);
  public static final double noteDetectionLimelightCameraOffsetInDegreesYaw =
      Units.degreesToRadians(0);

  public static final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Camera offset from robot center. Camera A is on the left side of the robot from front view.
  public static final Transform3d robotToAprilTagLimelightCameraA =
      new Transform3d(
          new Translation3d(
              aprilTagLimelightCameraADistanceFromCenterX,
              aprilTagLimelightCameraADistanceFromCenterY,
              aprilTagLimelightCameraADistanceFromGroundZ),
          new Rotation3d(
              aprilTagLimelightCameraAOffsetInRadiansRoll,
              aprilTagLimelightCameraAOffsetInRadiansPitch,
              aprilTagLimelightCameraAOffsetInRadiansYaw));

  // Camera offset from robot center. Camera B is on the right side of the robot from front view.
  public static final Transform3d robotToAprilTagLimelightCameraB =
      new Transform3d(
          new Translation3d(
              aprilTagLimelightCameraBDistanceFromCenterX,
              aprilTagLimelightCameraBDistanceFromCenterY,
              aprilTagLimelightCameraBDistanceFromGroundZ),
          new Rotation3d(
              aprilTagLimelightCameraBOffsetInRadiansRoll,
              aprilTagLimelightCameraBOffsetInRadiansPitch,
              aprilTagLimelightCameraBOffsetInRadiansYaw));

  public static final double poseXTolerance = Units.inchesToMeters(4);
  public static final double poseYTolerance = Units.inchesToMeters(4);
  public static final double poseZTolerance = Units.inchesToMeters(4);
  public static final double posePitchTolerance = Units.degreesToRadians(4);
  public static final double poseRollTolerance = Units.degreesToRadians(4);
  public static final double poseYawTolerance = Units.degreesToRadians(4);

  public enum CAMERA_TYPE {
    LIMELIGHT,
    PHOTONVISION
  }

  public enum PIPELINE {
    APRILTAGS(0);
    private final int pipeline;

    PIPELINE(final int pipeline) {
      this.pipeline = pipeline;
    }

    public int get() {
      return pipeline;
    }
  }

  public enum CAMERA_SERVER {
    INTAKE("10.42.1.11"),
    LIMELIGHTA("10.42.1.12"),
    LIMELIGHTB("10.42.1.13"),
    ;
    private final String ip;

    CAMERA_SERVER(final String ip) {
      this.ip = ip;
    }

    @Override
    public String toString() {
      return ip;
    }
  }

  public enum TRACKING_STATE {
    NONE,
    SPEAKER,
    NOTE,
    PASSING
  }
  
  public static final double velocityShoot = 9.255586759; // Previously 11.1 m/s
}
