// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.vision;

import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Seconds;

import com.limelightvision.IMUMode;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Time;

public class CameraIOLimelight4 extends CameraIOLimelight {

  // Triggers
  private static final Trigger enabled = RobotModeTriggers.disabled().negate();

  private static final Trigger fmsConnected = new Trigger(RobotState::isFMSAttached);

  @AutoLogOutput private static final Trigger teleopOver = RobotModeTriggers.teleop().negate();

  @AutoLogOutput
  private static final Trigger usingRewind =
      fmsConnected.or(() -> VisionConstants.useRewindOffField);

  // The current mode of the IMU, assuming this is not set anywhere else and IMU is enabled.
  private IMUMode currentMode_ =
      VisionConstants.useIMU ? IMUMode.EXTERNAL_SEED_INTERNAL : IMUMode.EXTERNAL;

  // The current throttle of the LL4, assuming this is not set anywhere else and throttling is
  // enabled.
  private int currentThrottle_ =
      VisionConstants.regulateThrottle ? VisionConstants.numSkippedFramesDisabled : 0;

  public CameraIOLimelight4(String name, Supplier<Rotation2d> rotationSupplier) {
    super(name, rotationSupplier);

    camera.setRewindEnabled(VisionConstants.useRewind);

    bindIMUCommands();
    bindThrottleCommands();
    bindRewindCommands();
  }

  @Override
  public void updateInputs(CameraIOInputsAutoLogged inputs) {
    super.updateInputs(inputs);

    camera.setIMUMode(currentMode_);
    camera.setThrottle(currentThrottle_);

    inputs.imuMode = currentMode_;
    inputs.imuRobotYaw = Rotation2d.fromDegrees(camera.getIMUData().yawDegrees);
    inputs.frameSkipThrottle = currentThrottle_;
  }

  private void bindThrottleCommands() {
    // If not throttling, skip binding these commands.
    if (!VisionConstants.regulateThrottle) return;

    enabled.whileTrue(setThrottleCommand(VisionConstants.numSkippedFramesEnabled));
    enabled.whileFalse(setThrottleCommand(VisionConstants.numSkippedFramesDisabled));
  }

  private void bindIMUCommands() {
    // If running without IMU, skip binding these commands.
    if (!VisionConstants.useIMU) return;

    // When enabled, set to use IMU after slight offset.
    enabled.onTrue(
        setModeCommand(VisionConstants.enabledIMUMode)
            .beforeStarting(Commands.waitTime(Milliseconds.of(100))));

    // When disabled, use gryo to reset IMU.
    enabled.onFalse(setModeCommand(IMUMode.EXTERNAL_SEED_INTERNAL));
  }

  private void bindRewindCommands() {
    // If rewind is disabled, skip binding these commands.
    if (!VisionConstants.useRewind) return;

    // When the robot disables, save the rewind.
    teleopOver
        .and(usingRewind)
        .onTrue(
            saveRewind(Seconds.of(160))
                .finallyDo(() -> System.out.println("Limelight rewind saved!"))
                .ignoringDisable(true));
  }

  private void setThrottle(int throttle) {
    currentThrottle_ = throttle;
  }

  private Command setThrottleCommand(int throttle) {
    return Commands.runOnce(() -> setThrottle(throttle)).ignoringDisable(true);
  }

  private void setMode(IMUMode mode) {
    currentMode_ = mode;
  }

  private Command setModeCommand(IMUMode mode) {
    return Commands.runOnce(() -> setMode(mode)).ignoringDisable(true);
  }

  private Command saveRewind(Time duration) {
    return Commands.runOnce(() -> camera.triggerRewindCapture(duration.in(Seconds)));
  }
}
