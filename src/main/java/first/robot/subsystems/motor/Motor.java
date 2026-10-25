// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.motor;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

public class Motor extends SubsystemBase {

  private final MotorIO io;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  public Motor(MotorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Motor", inputs);
  }

  /**
   * A command that runs the motor at a certain voltage and stops when done
   *
   * @param volts
   * @return
   */
  public Command runVoltage(Voltage volts) {
    return startEnd(() -> io.setVoltage(volts), io::stop);
  }

  /**
   * A command that run the motor at a certain velocity and stops when done
   *
   * @param velocity
   * @return
   */
  public Command runVelocity(AngularVelocity velocity) {
    return startEnd(() -> io.setVelocity(velocity), io::stop);
  }
}
