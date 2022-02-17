package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera("limelight");
  private PhotonPipelineResult result = camera.getLatestResult();

  public double getRotation() {
    if (!result.hasTargets()) {
      return 0;
    }
    return result.getBestTarget().getYaw();
  }

  public double getDistance() {
    if (!result.hasTargets()) {
      return 0;
    }

    return PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT,
        TARGET_HEIGHT,
        CAMERA_PITCH,
        Units.degreesToRadians(result.getBestTarget().getPitch()));
  }

  public Transform2d getTransform() {
    if (!result.hasTargets()) {
      return new Transform2d();
    }

    return result.getBestTarget().getCameraToTarget();
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();
  }
}
