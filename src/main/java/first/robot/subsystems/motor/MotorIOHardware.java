// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.motor;

import static org.wpilib.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import first.robot.util.PhoenixUtil;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class MotorIOHardware implements MotorIO {
  private final TalonFX motor;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> current;

  public MotorIOHardware(int canid, CANBus bus) {
    motor = new TalonFX(canid, bus);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    voltage = motor.getMotorVoltage();
    current = motor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), position, velocity, voltage, current);

    motor.optimizeBusUtilization();

    Slot0Configs config = new Slot0Configs();

    config.kP = 2.0;
    config.kI = 0.0;
    config.kD = 0.0;

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    var status = BaseStatusSignal.refreshAll(position, velocity, voltage, current);

    inputs.connected = status.isOK();

    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.voltage = voltage.getValue();
    inputs.current = current.getValue();
  }

  @Override
  public void setVoltage(Voltage volts) {
    motor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(AngularVelocity vel) {
    motor.setControl(new VelocityVoltage(vel));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
