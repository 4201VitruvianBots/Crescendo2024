package frc.robot.constants;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Map;

public final class AUTO {
  public enum START_POSE {
    NONE(new Pose2d(-1, -1, new Rotation2d())),
    ONE_WAIT_AUTO("SimpleAuto1"),
    FOUR_PIECE_NEAR("FourPiecePt1"),
    FIVE_PIECE("5Piecept1"),
    TWO_PIECE("2PieceNearPart1"),
    TWO_PIECE_FAR("TwoPieceFarPt1");
//    THREE_PIECE_FAR(new Pose2d(1.40, 2.36, new Rotation2d()));

    private final Pose2d pose;

    START_POSE(final String startPathName) {
      var path = PathPlannerPath.fromPathFile(startPathName);

      this.pose = path.getPreviewStartingHolonomicPose();
    }

    START_POSE(final Pose2d pose) {
      this.pose = pose;
    }

    public Pose2d get() {
      return pose;
    }
  }

  public static final Map<String, START_POSE> AUTO_POSE_MAP =
      Map.ofEntries(
          Map.entry("OneWaitAuto", START_POSE.ONE_WAIT_AUTO),
          Map.entry("FourPieceNear", START_POSE.FOUR_PIECE_NEAR),
          Map.entry("FivePiece", START_POSE.FIVE_PIECE),
          Map.entry("TwoPieceAuto", START_POSE.TWO_PIECE),
          Map.entry("TwoPieceFar", START_POSE.TWO_PIECE_FAR)
          // Map.entry("ThreePieceFar", START_POSE.THREE_PIECE_FAR),
          );
}
