// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.motor;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public interface MotorIO {
  @AutoLog
  public static class MotorInputs {
    public Angle position = Rotations.zero();
    public AngularVelocity velocity = RotationsPerSecond.zero();

    public Voltage voltage = Volts.zero();
    public Current current = Amps.zero();

    public boolean connected = false;
  }

  public default void updateInputs(MotorInputs inputs) {}

  public default void setVoltage(Voltage volts) {}

  public default void setVelocity(AngularVelocity vel) {}

  public default void stop() {}
}
