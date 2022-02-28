package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Comparator;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera("limelight");
  private PhotonPipelineResult result = camera.getLatestResult();

  @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
  private Optional<PhotonTrackedTarget> target =
      result.getTargets().stream().min(Comparator.comparing(PhotonTrackedTarget::getYaw));

  public double getYaw() {
    return target.map(PhotonTrackedTarget::getYaw).orElse(0.0);
  }

  public double getDistance() {
    return target.isPresent()
        ? PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT,
            TARGET_HEIGHT,
            CAMERA_PITCH,
            Units.degreesToRadians(result.getBestTarget().getPitch()))
        : 0;
  }

  public boolean hasTargets() {
    return result.hasTargets();
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();
    target = result.getTargets().stream().min(Comparator.comparing(PhotonTrackedTarget::getYaw));
    SmartDashboard.putBoolean("hasTargets", result.hasTargets());
  }
}
