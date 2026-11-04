// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.i2Csensor;

import com.revrobotics.ColorSensorV3;
import org.wpilib.hardware.bus.I2C.Port;

public class I2csensorhardware implements I2CsensorIO {
  private final ColorSensorV3 sensor;

  public I2csensorhardware(Port port) {
    sensor = new ColorSensorV3(port);
  }

  @Override
  public void updateInputs(I2CsensorInputs inputs) {
    inputs.isConnected = sensor.isConnected();
    inputs.redValue = sensor.getRed();
    inputs.blueValue = sensor.getBlue();
    inputs.greenValue = sensor.getGreen();
    inputs.proximity = sensor.getProximity();
  }
}
