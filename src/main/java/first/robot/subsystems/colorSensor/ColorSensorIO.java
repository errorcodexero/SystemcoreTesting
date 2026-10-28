// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.colorSensor;

import com.revrobotics.ColorSensorV3;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.util.Color;

public interface ColorSensorIO {

  @AutoLog
  public static class ColorInputs {
    public Color detectedColor1 = new Color();
    public int proximity1 = 0;
    public double IR1 = 0;
    public boolean isConnected1 = false;

    public Color detectedColor0 = new Color();
    public int proximity0 = 0;
    public double IR0 = 0;
    public boolean isConnected0 = false;
  }

  // Gets the inputs from the color sensor on port 1
  public default void updateColorInputs1(ColorInputs inputs, ColorSensorV3 colorSensor) {}

  // Gets the inputs from the color sensor on port 0
  public default void updateColorInputs0(ColorInputs inputs, ColorSensorV3 colorSensor) {}
}
