// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.colorsensor;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.util.Color;

public interface ColorSensorIO {
  @AutoLog
  public static class ColorSensorInputs {
    public boolean connected = false;

    public int[] rawColor = {0, 0, 0, 0}; // red, green, blue, ir
    public Color color = Color.BLACK;
    public int proximity = 0;
  }

  public default void updateInputs(ColorSensorInputs inputs) {}
}
