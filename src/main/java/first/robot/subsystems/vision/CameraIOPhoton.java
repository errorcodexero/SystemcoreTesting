// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.vision;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Percent;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

public class CameraIOPhoton implements CameraIO {

  // Transform from robot to camera.
  protected final Transform3d robotToCamera_;
  protected final PhotonCamera camera_;

  private final int id_;
  private static int nextId = 0;

  public CameraIOPhoton(String name, Transform3d robotToCamera) {
    // Setup camera
    camera_ = new PhotonCamera(name);
    robotToCamera_ = robotToCamera;

    id_ = nextId;
    nextId++;
  }

  @Override
  public void updateInputs(CameraIOInputsAutoLogged inputs) {

    // Connected Status
    inputs.connected = camera_.isConnected();

    // Camera Name
    inputs.name = camera_.getName();

    // Results
    List<PhotonPipelineResult> results = camera_.getAllUnreadResults();
    List<PoseEstimation> estimations = new ArrayList<>();

    for (PhotonPipelineResult result : results) {

      if (!result.hasTargets()) {
        inputs.simpleID = 0;
        inputs.simpleX = Degrees.zero();
        inputs.simpleY = Degrees.zero();
        inputs.simpleArea = Percent.zero();
        inputs.simpleValid = false;
        inputs.fiducials = new Fiducial[] {};
        inputs.rawCorners = new Translation2d[] {};

        continue;
      }

      // Get best target

      PhotonTrackedTarget bestTarget = result.getBestTarget();

      // Target information to fill

      ArrayList<Translation2d> cornerCoords = new ArrayList<>();
      ArrayList<Fiducial> fiducials = new ArrayList<>();

      // Get target information

      for (PhotonTrackedTarget target : result.getTargets()) {

        for (TargetCorner corner : target.getDetectedCorners()) {
          cornerCoords.add(new Translation2d(corner.x, corner.y));
        }

        fiducials.add(
            new Fiducial(
                target.getFiducialId(), target.getArea(), target.getPitch(), target.getYaw()));
      }

      // Calculate Average Tag Distance and Ambiguity

      double averageTagDist = 0.0;

      for (PhotonTrackedTarget target : result.targets) {
        averageTagDist += target.getBestCameraToTarget().getTranslation().getNorm();
      }

      averageTagDist /= result.targets.size();

      // Get Pose Estimates

      Optional<MultiTargetPNPResult> multitagResult = result.multitagResult;

      if (multitagResult.isPresent()) {
        Transform3d fieldToCamera = multitagResult.get().estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera_.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        estimations.add(
            new PoseEstimation(
                robotPose.toPose2d(),
                result.getTimestampSeconds(),
                averageTagDist,
                multitagResult.get().estimatedPose.ambiguity,
                multitagResult.get().fiducialIDsUsed.size(),
                PoseEstimationType.PHOTON_MULTITAG,
                id_,
                true));
      }

      inputs.simpleID = bestTarget.getFiducialId();
      inputs.simpleX = Degrees.of(bestTarget.getPitch());
      inputs.simpleY = Degrees.of(bestTarget.getYaw());
      inputs.simpleArea = Percent.of(bestTarget.getArea());
      inputs.simpleValid = true;

      inputs.rawCorners = cornerCoords.toArray(new Translation2d[0]);
      inputs.fiducials = fiducials.toArray(new Fiducial[0]);
    }

    inputs.poseEstimates = estimations.toArray(new PoseEstimation[0]);
  }

  @Override
  public String getName() {
    return camera_.getName();
  }

  @Override
  public void forceBlink() {
    camera_.setLED(VisionLEDMode.kBlink);
  }

  @Override
  public void forceOff() {
    camera_.setLED(VisionLEDMode.kOff);
  }

  @Override
  public void forceOn() {
    camera_.setLED(VisionLEDMode.kOn);
  }

  @Override
  public void resetLed() {
    camera_.setLED(VisionLEDMode.kDefault);
  }
}
