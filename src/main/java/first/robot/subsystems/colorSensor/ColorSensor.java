// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.colorSensor;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;

public class ColorSensor extends SubsystemBase {

  private final ColorInputsAutoLogged loggedInputs = new ColorInputsAutoLogged();

  private final ColorSensorIO io;

  public ColorSensor(ColorSensorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateColorInputsPort1(loggedInputs);
    Logger.processInputs("ColorSensorInputs", loggedInputs);
  }
}
