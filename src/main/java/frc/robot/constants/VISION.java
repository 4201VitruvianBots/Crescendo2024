// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.constants;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.util.Units;

// public final class VISION {
//   public static final double aprilTagLimelightCameraADistanceFromCenterX =
//       Units.inchesToMeters(-7.590951);
//   public static final double aprilTagLimelightCameraADistanceFromCenterY =
//       Units.inchesToMeters(-9.541729);
//   public static final double aprilTagLimelightCameraADistanceFromGroundZ =
//       Units.inchesToMeters(15.184963);
//   public static final double aprilTagLimelightCameraAOffsetInRadiansRoll =
//       Units.degreesToRadians(0);
//   public static final double aprilTagLimelightCameraAOffsetInRadiansPitch =
//       Units.degreesToRadians(6.172091);
//   public static final double aprilTagLimelightCameraAOffsetInRadiansYaw =
//       Units.degreesToRadians(-145.513393);

//   public static final double aprilTagLimelightCameraBDistanceFromCenterX =
//       Units.inchesToMeters(-7.590951);
//   public static final double aprilTagLimelightCameraBDistanceFromCenterY =
//       Units.inchesToMeters(9.541729);
//   public static final double aprilTagLimelightCameraBDistanceFromGroundZ =
//       Units.inchesToMeters(15.184963);
//   public static final double aprilTagLimelightCameraBOffsetInRadiansRoll =
//       Units.degreesToRadians(0);
//   public static final double aprilTagLimelightCameraBOffsetInRadiansPitch =
//       Units.degreesToRadians(6.172091);
//   public static final double aprilTagLimelightCameraBOffsetInRadiansYaw =
//       Units.degreesToRadians(145.513393);

//   public static final double noteDetectionLimelightCameraDistanceFromCenterX =
//       Units.inchesToMeters(12.125);
//   public static final double noteDetectionLimelightCameraDistanceFromCenterY =
//       Units.inchesToMeters(0);
//   public static final double noteDetectionLimelightCameraDistanceFromGroundZ =
//       Units.inchesToMeters(7.040388);
//   public static final double noteDetectionLimelightCameraOffsetInDegreesRoll =
//       Units.degreesToRadians(0);
//   public static final double noteDetectionLimelightCameraOffsetInDegreesPitch =
//       Units.degreesToRadians(0);
//   public static final double noteDetectionLimelightCameraOffsetInDegreesYaw =
//       Units.degreesToRadians(0);

//   public static final AprilTagFieldLayout aprilTagFieldLayout =
//       AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

//   // Camera offset from robot center. Camera A is on the left side of the robot from front view.
//   public static final Transform3d robotToAprilTagLimelightCameraA =
//       new Transform3d(
//           new Translation3d(
//               aprilTagLimelightCameraADistanceFromCenterX,
//               aprilTagLimelightCameraADistanceFromCenterY,
//               aprilTagLimelightCameraADistanceFromGroundZ),
//           new Rotation3d(
//               aprilTagLimelightCameraAOffsetInRadiansRoll,
//               aprilTagLimelightCameraAOffsetInRadiansPitch,
//               aprilTagLimelightCameraAOffsetInRadiansYaw));

//   // Camera offset from robot center. Camera A is on the left side of the robot from front view.
//   public static final Transform3d robotToAprilTagLimelightCameraB =
//       new Transform3d(
//           new Translation3d(
//               aprilTagLimelightCameraBDistanceFromCenterX,
//               aprilTagLimelightCameraBDistanceFromCenterY,
//               aprilTagLimelightCameraBDistanceFromGroundZ),
//           new Rotation3d(
//               aprilTagLimelightCameraAOffsetInRadiansRoll,
//               aprilTagLimelightCameraAOffsetInRadiansPitch,
//               aprilTagLimelightCameraAOffsetInRadiansYaw));

//   public enum CAMERA_TYPE {
//     LIMELIGHT,
//     PHOTONVISION
//   }

//   public enum PIPELINE {
//     APRILTAGS(0);
//     private final int pipeline;

//     PIPELINE(final int pipeline) {
//       this.pipeline = pipeline;
//     }

//     public int get() {
//       return pipeline;
//     }
//   }

//   public enum CAMERA_SERVER {
//     FRONT("10.42.1.11");
//     private final String ip;

//     CAMERA_SERVER(final String ip) {
//       this.ip = ip;
//     }

//     @Override
//     public String toString() {
//       return ip;
//     }
//   }
// }
