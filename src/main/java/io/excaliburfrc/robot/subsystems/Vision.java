package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera("limelight");
  private PhotonPipelineResult result = camera.getLatestResult();

  public Transform2d getTransform() {
    return result.hasTargets() ? result.getBestTarget().getCameraToTarget() : new Transform2d();
  }

  public boolean noTargets() {
    return !result.hasTargets();
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();
    SmartDashboard.putBoolean("hasTargets", result.hasTargets());
  }
}
