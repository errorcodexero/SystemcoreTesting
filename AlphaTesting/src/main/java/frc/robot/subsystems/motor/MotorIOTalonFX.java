package frc.robot.subsystems.motor;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class MotorIOTalonFX implements MotorIO {

    private final TalonFX talonfx;

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Angle> position;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;

    public MotorIOTalonFX(int canid, CANBus canbus) {
        talonfx = new TalonFX(canid, canbus);

        // Get status signals
        velocity = talonfx.getVelocity();
        position = talonfx.getPosition();
        voltage = talonfx.getMotorVoltage();
        current = talonfx.getStatorCurrent();

        // Set their update frequency and optimize CAN usage.
        PhoenixUtil.tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), velocity, position, voltage, current));
        PhoenixUtil.tryUntilOk(5, talonfx::optimizeBusUtilization);

        // Create its configuration
        var config = new Slot0Configs()
            .withKP(1.0);

        // Apply its configuration
        PhoenixUtil.tryUntilOk(5, () -> talonfx.getConfigurator().apply(config, 0.25));
    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        BaseStatusSignal.refreshAll(velocity, position, voltage, current);

        inputs.velocity = velocity.getValue();
        inputs.position = position.getValue();
        inputs.voltage = voltage.getValue();
        inputs.current = current.getValue();
    }

    @Override
    public void runPosition(Angle position) {
        talonfx.setControl(new PositionVoltage(position));
    }

    @Override
    public void runVelocity(AngularVelocity velocity) {
        talonfx.setControl(new VelocityVoltage(velocity));
    }

    @Override
    public void stop() {
        talonfx.stopMotor();
    }
}
