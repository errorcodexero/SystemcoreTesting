// Copyright (c) 2026 FRC 1425 Error Code Xero 
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

import com.limelightvision.FiducialTarget;
import com.limelightvision.LEDMode;
import com.limelightvision.Limelight;
import com.limelightvision.Status;

public class CameraIOLimelight implements CameraIO {

  protected Supplier<Rotation2d> rotationSupplier_;

  private final Limelight camera;

  private final int id_;
  private static int nextId = 0;

  public CameraIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    rotationSupplier_ = rotationSupplier;
    camera = new Limelight(name);

    id_ = nextId;
    nextId++;
  }

  @Override
  public void updateInputs(CameraIOInputsAutoLogged inputs) {

    if(id_ == 0){
      Limelight.setSharedRobotOrientation(rotationSupplier_.get().getDegrees());
    }

    inputs.connected = camera.getStatus() == Status.OK;
    inputs.name = camera.getName();
    inputs.cpuTemp = camera.getHardwareData().cpuTempCelsius;
    //TODO: Find way to get FPS

    inputs.simpleID = -1; //TODO: DO THIS
    inputs.simpleArea = camera.getTargetAreaPercent();
    inputs.simpleX = camera.getTXDegrees();
    inputs.simpleY = camera.getTYDegrees();
    inputs.simpleValid = camera.hasTarget();

    ArrayList<Translation2d> corners = new ArrayList<>();
    ArrayList<Fiducial> fiducials = new ArrayList<>();

    var rawFiducials = camera.getLatestResults().fiducialTargets;
    for(FiducialTarget fid : rawFiducials){
      fiducials.add(new Fiducial(fid.fiducialId, fid.targetAreaPercent, fid.txDegrees, fid.tyDegrees));
      
      for(double[] corner : fid.corners){
        corners.add(new Translation2d(corner[0], corner[1]));
      }
    }

    inputs.rawCorners = corners.toArray(new Translation2d[0]);
    inputs.fiducials = fiducials.toArray(new Fiducial[0]);

    

    // PoseEstimate estimateMegatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name_);

    // if (estimateMegatag2 != null && estimateMegatag2.tagCount > 0) {
    //   inputs.poseEstimate =
    //       new PoseEstimation(
    //           estimateMegatag2.pose,
    //           estimateMegatag2.timestampSeconds,
    //           estimateMegatag2.avgTagDist,
    //           0.0,
    //           estimateMegatag2.tagCount,
    //           PoseEstimationType.MEGATAG2,
    //           id_,
    //           true);
    // } else {
    //   inputs.poseEstimate =
    //       new PoseEstimation(Pose2d.kZero, 0, 0, 0, 0, PoseEstimationType.MEGATAG2, id_, false);
    // }
  }

  @Override
  public String getName() {
    return camera.getName();
  }

  @Override
  public void forceBlink() {
    camera.setLEDMode(LEDMode.FORCE_BLINK);
  }

  @Override
  public void forceOff() {
    camera.setLEDMode(LEDMode.FORCE_OFF);  
  }

  @Override
  public void forceOn() {
    camera.setLEDMode(LEDMode.FORCE_ON);  
  }

  @Override
  public void resetLed() {
    camera.setLEDMode(LEDMode.PIPELINE_CONTROL);  
  }
}
