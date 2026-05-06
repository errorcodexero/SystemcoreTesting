package frc.robot.subsystems.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface MotorIO {
    @AutoLog
    public static class MotorInputs {
        Angle position = Radians.zero();
        AngularVelocity velocity = RadiansPerSecond.zero();
        Voltage voltage = Volts.zero();
        Current current = Amps.zero();
    }

    public default void runVelocity(AngularVelocity velocity) {};
    public default void runPosition(Angle position) {};
    public default void stop() {};

    public default void updateInputs(MotorInputs inputs) {};
}
