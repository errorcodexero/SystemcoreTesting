// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.colorSensor;

import com.revrobotics.ColorSensorV3;
import org.wpilib.hardware.bus.I2C;
import org.wpilib.util.Color;

@SuppressWarnings("unused")
public class ColorSensorIOHardware implements ColorSensorIO {

  //port 1
  private Color detectedColor1 = new Color();
  private int proximity1 = 0;
  private double IR1 = 0;
  private boolean isConnected1 = false;

  //port 2
  private Color detectedColor0 = new Color();
  private int proximity0 = 0;
  private double IR0 = 0;
  private boolean isConnected0 = false;

  public ColorSensorIOHardware() {
    final ColorSensorV3 hardwareColorSensorPort1 = new ColorSensorV3(I2C.Port.PORT_1);
    final ColorSensorV3 hardwareColorSensorPort0 = new ColorSensorV3(I2C.Port.PORT_0);

    detectedColor1 = hardwareColorSensorPort1.getColor();
    proximity1 = hardwareColorSensorPort1.getProximity();
    IR1 = hardwareColorSensorPort1.getIR();
    isConnected1 = hardwareColorSensorPort1.isConnected();

    detectedColor0 = hardwareColorSensorPort0.getColor();
    proximity0 = hardwareColorSensorPort0.getProximity();
    IR0 = hardwareColorSensorPort0.getIR();
    isConnected0 = hardwareColorSensorPort0.isConnected();
  }

  @Override
  public void updateColorInputs1(ColorInputs inputs, ColorSensorV3 colorSensor) {
    inputs.IR1 = colorSensor.getIR();
    inputs.detectedColor1 = colorSensor.getColor();
    inputs.proximity1 = colorSensor.getProximity();
    inputs.isConnected1 = colorSensor.isConnected();
  }

  @Override
  public void updateColorInputs0(ColorInputs inputs, ColorSensorV3 colorSensor) {
    inputs.IR0 = colorSensor.getIR();
    inputs.detectedColor0 = colorSensor.getColor();
    inputs.proximity0 = colorSensor.getProximity();
    inputs.isConnected0 = colorSensor.isConnected();

  }
  
}
