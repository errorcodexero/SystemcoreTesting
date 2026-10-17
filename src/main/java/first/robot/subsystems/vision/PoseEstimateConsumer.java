// Copyright (c) 2026 FRC 1425 Error Code Xero 
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.vision;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

@FunctionalInterface
public interface PoseEstimateConsumer {
  public void integrate(
      Pose2d robotPose, double timestampSeconds, Matrix<N3, N1> standardDeviations);

  public static PoseEstimateConsumer ignore() {
    return (Pose2d robotPose, double timestampSecnds, Matrix<N3, N1> standardDeviations) -> {};
  }
}
