package frc.robot.subsystems;

import static frc.robot.utils.TestUtils.refreshAkitData;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;

public class TestControls {

  Controls m_controls = new Controls();

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    /* enable the robot */
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    refreshAkitData();

    /* delay ~100ms so the devices can start up and enable */
    Timer.delay(0.100);
  }

  @Disabled("Issues with AdvantageKit implementation. Test with Simulation")
  public void testAutoStartPoseFlip() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    refreshAkitData();
    m_controls.periodic();
    m_controls.updateStartPose("FourPieceNear");

    PathPlannerPath path1 = PathPlannerPath.fromPathFile("fourpiecept1");

    assertEquals(
        path1.getPreviewStartingHolonomicPose().getTranslation(),
        m_controls.getStartPose().getTranslation());

    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    refreshAkitData();
    m_controls.periodic();
    m_controls.updateStartPose("FourPieceNear");

    var flippedPath = path1.flipPath();

    assertEquals(
        flippedPath.getPreviewStartingHolonomicPose().getTranslation(),
        m_controls.getStartPose().getTranslation());
  }
}
