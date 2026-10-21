// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.vision;

import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Microseconds;
import static org.wpilib.units.Units.Seconds;

import com.limelightvision.FiducialTarget;
import com.limelightvision.LEDMode;
import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;
import com.limelightvision.Status;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.Time;

public class CameraIOLimelight implements CameraIO {

  protected final Limelight camera;
  protected final Supplier<Rotation2d> rotationSupplier_;
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

    // Set robot orientation only once per robot loop.
    if (id_ == 0) {
      Limelight.setSharedRobotOrientation(rotationSupplier_.get().getDegrees());
    }

    inputs.connected = camera.getStatus() == Status.OK;
    inputs.name = camera.getName();
    inputs.cpuTemp = camera.getHardwareData().cpuTempCelsius;
    inputs.fps = calculateFPS().in(Hertz);

    inputs.simpleID = -1; // TODO: Get the ID of simple ID or dont have this input
    inputs.simpleArea = camera.getTargetAreaPercent();
    inputs.simpleX = camera.getTXDegrees();
    inputs.simpleY = camera.getTYDegrees();
    inputs.simpleValid = camera.hasTarget();

    ArrayList<Translation2d> corners = new ArrayList<>();
    ArrayList<Fiducial> fiducials = new ArrayList<>();

    var rawFiducials = camera.getLatestResults().fiducialTargets;
    for (FiducialTarget fid : rawFiducials) {
      fiducials.add(
          new Fiducial(fid.fiducialId, fid.targetAreaPercent, fid.txDegrees, fid.tyDegrees));

      for (double[] corner : fid.corners) {
        corners.add(new Translation2d(corner[0], corner[1]));
      }
    }

    inputs.rawCorners = corners.toArray(new Translation2d[0]);
    inputs.fiducials = fiducials.toArray(new Fiducial[0]);

    ArrayList<PoseEstimation> estimates = new ArrayList<>();

    var estimateQueue = camera.readPoseEstimateQueue(PoseEstimateType.MT2_WPIBLUE);

    // TODO: also include megatag1 for logging purposes
    for (PoseEstimate estimate : estimateQueue) {
      estimates.add(
          new PoseEstimation(
              estimate.pose,
              estimate.timestampSeconds,
              estimate.avgTagDistanceMeters,
              0.0,
              estimate.fieldedTagCount,
              PoseEstimationType.MEGATAG2,
              id_,
              true));
    }
  }

  // Variables for FPS Calculations
  private long lastFrameIndex = 0;
  private Time lastFrameTimestamp = Seconds.zero();
  private Frequency lastFPS = Hertz.zero();

  /**
   * Calculate the average FPS since the last call of this method.
   *
   * @return
   */
  private Frequency calculateFPS() {
    LimelightResults results = camera.getLatestResults();

    long newFrameIndex = results.frameIndex;
    Time newTimestamp = Microseconds.of(results.timestampMicros);

    // First call, or the camera's counter reset: just resync.
    if (lastFrameIndex < 0 || newFrameIndex < lastFrameIndex) {
      lastFrameIndex = newFrameIndex;
      lastFrameTimestamp = newTimestamp;
      return lastFPS;
    }

    long framesPassed = results.frameIndex - lastFrameIndex;
    Time timePassed = newTimestamp.minus(lastFrameTimestamp);
    Time timePerFrame = timePassed.div(framesPassed);

    // No new frame (or no time elapsed): keep the last known value.
    if (framesPassed == 0 || timePassed.lte(Seconds.zero())) {
      return lastFPS;
    }

    lastFPS = timePerFrame.asFrequency();
    lastFrameIndex = newFrameIndex;
    lastFrameTimestamp = newTimestamp;

    return lastFPS;
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
