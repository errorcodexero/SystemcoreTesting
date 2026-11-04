// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.i2c;

import org.littletonrobotics.junction.AutoLog;

public interface I2CsensorIO {
  @AutoLog
  public static class I2CsensorInputs {
    public boolean isConnected = false;
    public double redValue = 0.0;
    public double greenValue = 0.0;
    public double blueValue = 0.0;
    public int proximity = 0;
  }

  public default void updateInputs(I2CsensorInputs inputs) {}
}
