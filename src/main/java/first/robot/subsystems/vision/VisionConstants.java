// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.vision;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import com.limelightvision.IMUMode;
import first.robot.subsystems.vision.AprilTagVision.IntegrationBehavior;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

public class VisionConstants {

  // Limelight Names
  public static final String frontLimelightName = "limelight-front";

  // Behavior to use when inegrating pose estimates.
  public static final IntegrationBehavior integrationBehavior = IntegrationBehavior.ONLY_NEAREST;

  // Transforms
  public static final Transform3d frontTransform =
      new Transform3d(
          new Translation3d(Meters.of(0.0), Meters.of(0.0), Meters.of(0.526)),
          new Rotation3d(Degrees.zero(), Degrees.of(-28), Degrees.zero()));

  public static final Transform3d backLeftTransform =
      new Transform3d(
          new Translation3d(Meters.of(-0.160), Meters.of(0.322), Meters.of(0.216)),
          new Rotation3d(Degrees.zero(), Degrees.of(-30), Degrees.of(150)));

  public static final Transform3d backRightTransform =
      new Transform3d(
          new Translation3d(Meters.of(-0.162), Meters.of(-0.322), Meters.of(0.216)),
          new Rotation3d(Degrees.zero(), Degrees.of(-30), Degrees.of(210)));

  // Odometry Filtering Configuration
  public static final int minimumTagCount = 1;
  public static final double maximumAmbiguity = 0.3; // For Photonvision Sim

  // Standard Deviation Factors, (we need to talk about how stddev is calculated, dont change until
  // then)
  public static final double baseLinearStdDev = 0.02;
  public static final double baseAngularStdDev = 0.06;
  public static final double megatag2Factor = 0.7;

  // LL4 Config
  public static final boolean useIMU = false; // Whether or not to use the internal IMU
  public static final IMUMode enabledIMUMode =
      IMUMode.INTERNAL_EXTERNAL_ASSIST; // What mode to use on the internal IMU

  public static final boolean useRewind = true; // Whether or not to enable rewind
  public static final boolean useRewindOffField =
      true; // Whether to enable rewind when not connected to the FMS.

  public static final boolean regulateThrottle =
      false; // Whether or not we use LL4 throttle to regulate temperature
  public static final int numSkippedFramesEnabled =
      0; // How many frames to skip in enabled, (probably 0 but you might want to?)
  public static final int numSkippedFramesDisabled =
      100; // Regulate temperature by skipping frames in disabled
}
