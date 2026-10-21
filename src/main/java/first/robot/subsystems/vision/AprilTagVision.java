// Copyright (c) 2026 FRC 1425 Error Code Xero 
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.vision;

import static org.wpilib.units.Units.Meters;

import first.robot.Constants.FieldConstants;
import first.robot.subsystems.vision.CameraIO.Fiducial;
import first.robot.subsystems.vision.CameraIO.PoseEstimation;
import first.robot.subsystems.vision.CameraIO.PoseEstimationType;
import first.robot.util.LoggedTracer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.units.measure.Distance;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

public class AprilTagVision extends SubsystemBase {

  static final Distance kDefaultDistanceFilter = Meters.of(100.0);

  public static enum IntegrationBehavior {
    ONLY_NEAREST,
    ALL
  }

  private final PoseEstimateConsumer poseEstimateConsumer_;

  private final CameraIO[] io_;
  private final CameraIOInputsAutoLogged[] inputs_;

  private final Alert[] alerts_;

  private boolean enabled_; // Whether or not vision pose estimation is enabled

  private Distance maxTagDistance = kDefaultDistanceFilter;

  public AprilTagVision(PoseEstimateConsumer poseEstimateConsumer, CameraIO... io) {
    poseEstimateConsumer_ = poseEstimateConsumer;

    io_ = io;
    enabled_ = true;

    inputs_ = new CameraIOInputsAutoLogged[io.length];
    alerts_ = new Alert[io.length];

    for (int i = 0; i < io.length; i++) {
      // Setup inputs for every camera
      inputs_[i] = new CameraIOInputsAutoLogged();

      // Setup alerts for every camera
      alerts_[i] =
          new Alert(
              "cameraDown" + i,
              "Camera " + i + ", \"" + io_[i].getName() + "\" is not connected!",
              Level.HIGH);
    }
  }

  public void setTagFilterDistance(Distance d) {
    maxTagDistance = (d == null) ? kDefaultDistanceFilter : d;
  }

  /**
   * Sets whether or not the vision pose estimates get merged with the drivebase.
   *
   * @param enabled
   */
  public void setEnabled(boolean enabled) {
    enabled_ = enabled;
  }

  /**
   * Command that sets whether or not the vision pose estimates get merged with the drivebase.
   *
   * @param enabled
   */
  public Command setEnabledCommand(boolean enabled) {
    return runOnce(() -> setEnabled(enabled));
  }

  @Override
  public void periodic() {

    LoggedTracer.reset();

    // Update inputs for each camera
    for (int index = 0; index < io_.length; index++) {
      io_[index].updateInputs(inputs_[index]);
      Logger.processInputs("Vision/Camera" + index, inputs_[index]);
    }

    LoggedTracer.record("Vision/UpdateInputs");

    ArrayList<Pose3d> summaryTagPoses = new ArrayList<>();
    ArrayList<PoseEstimation> poseEstimates = new ArrayList<>();

    // Iterate cameras for logging and pose estimations.
    for (int cam = 0; cam < io_.length; cam++) {
      // Activate disconnected alert.
      alerts_[cam].set(!inputs_[cam].connected);

      // Skip this camera if it is not connected.
      if (!inputs_[cam].connected) continue;

      ArrayList<Pose3d> tagPoses = new ArrayList<>();

      for (Fiducial fid : inputs_[cam].fiducials) {
        Optional<Pose3d> pose = FieldConstants.field.getTagPose(fid.id());
        if (pose.isPresent()) {
          tagPoses.add(pose.get());
        }
      }

      // TODO: Find a better way to add estimates to the arraylist
      for (PoseEstimation est : inputs_[cam].poseEstimate) {
        poseEstimates.add(est);
      }

      // Add to summaries.
      summaryTagPoses.addAll(tagPoses);

      // Log camera-specific outputs.
      Logger.recordOutput(getCameraKey(cam, "TagPoses"), tagPoses.toArray(new Pose3d[0]));
    }

    LoggedTracer.record("Vision/GatherPoses");

    // Integrate pose estimates

    ArrayList<PoseEstimation> acceptedEstimates = new ArrayList<>();
    ArrayList<PoseEstimation> declinedEstimates = new ArrayList<>();

    for (PoseEstimation est : poseEstimates) {
      if (isEstimationAcceptable(est)) {
        acceptedEstimates.add(est);
      } else {
        declinedEstimates.add(est);
      }
    }

    LoggedTracer.record("Vision/FilterPoses");

    switch (VisionConstants.integrationBehavior) {
      case ONLY_NEAREST -> {
        Optional<PoseEstimation> nearestEstimate = findMinDistanceEstimate(acceptedEstimates);

        if (nearestEstimate.isPresent()) {
          for (PoseEstimation est : acceptedEstimates) {
            if (est.cameraId() == nearestEstimate.get().cameraId()) {
              integratePoseEstimate(est);
            }
          }

          Logger.recordOutput("Vision/Summary/CameraUsing", nearestEstimate.get().cameraId());
        } else {
          Logger.recordOutput("Vision/Summary/CameraUsing", -1);
        }
      }
      default -> {
        for (PoseEstimation est : acceptedEstimates) {
          integratePoseEstimate(est);
        }
      }
    }

    LoggedTracer.record("Vision/IntegratePoses");

    Logger.recordOutput("Vision/Summary/TagPoses", summaryTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/BotPoses/All", estimateListToPoseArray(poseEstimates));
    Logger.recordOutput(
        "Vision/Summary/BotPoses/Accepted", estimateListToPoseArray(acceptedEstimates));
    Logger.recordOutput(
        "Vision/Summary/BotPoses/Declined", estimateListToPoseArray(declinedEstimates));

    Logger.recordOutput("Vision/PoseEstimatesEnabled", enabled_);
  }

  /**
   * Integrates a pose estimation with the PoseEstimator.
   *
   * @param est
   */
  private void integratePoseEstimate(PoseEstimation est) {
    double linearStdDev = VisionConstants.megatag2Factor;
    double angularStdDev = Double.POSITIVE_INFINITY;

    poseEstimateConsumer_.integrate(
        est.pose(), est.timestamp(), VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
  }

  /**
   * Decides whether or not a pose estimation is deemed acceptable to use in the PoseEstimator.
   *
   * @param estimation
   * @return Whether or not to integrate the pose.
   */
  private boolean isEstimationAcceptable(PoseEstimation estimation) {
    // Decline for any of the following reasons:

    if (!enabled_) return false; // If vision pose estimation is disabled.

    if (!estimation.valid()) return false; // If its not a valid pose estimate

    if (estimation.tagCount() == 0) return false; // If there are no tags on the estimate.

    if (estimation.type() == PoseEstimationType.MEGATAG1) return false; // If it is using Megatag1.

    if (estimation.tagCount() < VisionConstants.minimumTagCount)
      return false; // If there are less than the configured minimum.

    if (estimation.pose().getTranslation().getNorm() == 0)
      return false; // If the estimate is at the origin exactly (unrealistic).

    if (!isPoseOnField(estimation.pose())) return false; // The pose is not on the field.

    if (estimation.ambiguity() > VisionConstants.maximumAmbiguity)
      return false; // It is ambiguous (photonvision especially)

    return true;
  }

  /**
   * Finds the minimum distance estimate from an arraylist of estimates.
   *
   * @param estimates
   * @return The estimate with the least minimum distance
   */
  private Optional<PoseEstimation> findMinDistanceEstimate(ArrayList<PoseEstimation> estimates) {
    if (estimates.size() <= 0) return Optional.empty();

    PoseEstimation minDistance = estimates.get(0);

    for (int i = 1; i < estimates.size(); i++) {
      PoseEstimation est = estimates.get(i);
      if (est.averageDist() < minDistance.averageDist()) {
        minDistance = est;
      }
    }

    if (Meters.of(minDistance.averageDist()).gt(maxTagDistance)) {
      //
      // If the tag is greater than a given distance away, ignore it.  This is useful
      // for station collect during autonomous.
      //
      return Optional.empty();
    }

    return Optional.of(minDistance);
  }

  private Pose2d[] estimateListToPoseArray(List<PoseEstimation> estimates) {
    return estimates.stream().map(PoseEstimation::pose).toList().toArray(new Pose2d[0]);
  }

  /**
   * Whether or not the provided pose is within the field boundaries.
   *
   * @param pose
   * @return
   */
  private boolean isPoseOnField(Pose2d pose) {
    boolean xIsValid = pose.getX() > 0.0 && pose.getX() < FieldConstants.field.getFieldLength();
    boolean yIsValid = pose.getY() > 0.0 && pose.getY() < FieldConstants.field.getFieldWidth();

    return xIsValid && yIsValid;
  }

  /**
   * Get the key of a camera-specific value.
   *
   * @param index The camera to get the path of.
   * @param subtable The subtable within the camera to get.
   * @return The key to be used for logging.
   */
  private String getCameraKey(int index, String subtable) {
    return "Vision/" + getCameraName(index) + "/" + subtable;
  }

  /**
   * Gets the name of the camera subtable for logging
   *
   * @param index
   * @return
   */
  private String getCameraName(int index) {
    return "Camera" + Integer.toString(index);
  }
}
