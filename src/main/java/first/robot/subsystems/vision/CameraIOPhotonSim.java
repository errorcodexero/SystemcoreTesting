// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.vision;

import static org.wpilib.units.Units.Degrees;

import first.robot.Constants;
import first.robot.Constants.FieldConstants;
import first.robot.Constants.Mode;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;

public class CameraIOPhotonSim extends CameraIOPhoton {
  private final VisionSystemSim sim_;
  private final PhotonCameraSim camSim_;
  private final SimCameraProperties camProps_;

  private final Supplier<Pose2d> robotPoseSupplier_;

  public CameraIOPhotonSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> robotPoseSupplier, boolean ll4) {
    super(name, robotToCamera);

    // Create vision sim
    sim_ = new VisionSystemSim(name);

    // Setup apriltags in sim
    sim_.addAprilTags(FieldConstants.field);

    // Properties for camera simulation
    camProps_ = SimCameraProperties.LL2_960_720();

    // LL4 Calibration Estimate
    if (ll4) {
      camProps_.setCalibration(1280, 960, new Rotation2d(Degrees.of(90)));
    }

    // Setup camera sim
    camSim_ = new PhotonCameraSim(camera_, camProps_);

    sim_.addCamera(camSim_, robotToCamera_);

    if (Constants.getMode() != Mode.REAL) {
      Logger.recordOutput("CameraPoses/" + name, robotToCamera);
    }

    robotPoseSupplier_ = robotPoseSupplier;
  }

  @Override
  public void updateInputs(CameraIOInputsAutoLogged inputs) {
    sim_.update(robotPoseSupplier_.get());
    super.updateInputs(inputs);
  }
}
