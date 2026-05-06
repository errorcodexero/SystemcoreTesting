package frc.robot.subsystems.motor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
    private final MotorIO io;
    private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

    public Motor(MotorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    public Command velocity(AngularVelocity velocity) {
        return startEnd(() -> runVelocitySetpoint(velocity), io::stop);
    }

    public Command position(Angle position) {
        return startEnd(() -> runPositionSetpoint(position), io::stop);
    }

    private void runVelocitySetpoint(AngularVelocity velocity) {
        io.runVelocity(velocity);
    }

    private void runPositionSetpoint(Angle position) {
        io.runPosition(position);
    }
}
