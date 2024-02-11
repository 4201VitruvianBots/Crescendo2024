package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Map;

public final class AUTO {
  public enum START_POSE {
    NONE(new Pose2d(-1, -1, new Rotation2d())),
    FOUR_PIECE_NEAR(new Pose2d(1.4, 4.15, new Rotation2d())),
    THREE_PIECE_FAR(new Pose2d(1.40, 2.36, new Rotation2d()));

    private final Pose2d pose;

    START_POSE(final Pose2d pose) {
      this.pose = pose;
    }

    public Pose2d get() {
      return pose;
    }
  }

  public static Map<String, START_POSE> AUTO_POSE_MAP =
      Map.ofEntries(
          Map.entry("FourPieceNear", START_POSE.FOUR_PIECE_NEAR),
          Map.entry("ThreePieceFar", START_POSE.THREE_PIECE_FAR));
}
