// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.colorsensor;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import org.wpilib.hardware.bus.I2C.Port;

public class ColorSensorIORev implements ColorSensorIO {
  private final ColorSensorV3 sensor;

  public ColorSensorIORev(Port i2CPort) {
    sensor = new ColorSensorV3(i2CPort);
  }

  @Override
  public void updateInputs(ColorSensorInputs inputs) {
    inputs.connected = sensor.isConnected();

    RawColor rawColor = sensor.getRawColor();

    inputs.rawColor[0] = rawColor.red;
    inputs.rawColor[1] = rawColor.green;
    inputs.rawColor[2] = rawColor.blue;
    inputs.rawColor[3] = rawColor.ir;
    inputs.color = sensor.getColor();
    inputs.proximity = sensor.getProximity();
  }
}
