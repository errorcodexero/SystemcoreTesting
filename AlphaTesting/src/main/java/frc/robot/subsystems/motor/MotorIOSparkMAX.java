package frc.robot.subsystems.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class MotorIOSparkMAX implements MotorIO {

    private final SparkMax sparkmax;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pid;

    private final SparkMaxConfig config = new SparkMaxConfig();        

    public MotorIOSparkMAX(int canid, MotorType motorType) {
        sparkmax = new SparkMax(0, canid, motorType);
        encoder = sparkmax.getEncoder();
        pid = sparkmax.getClosedLoopController();

        config.closedLoop
            .p(0.001)
            .i(0)
            .d(0)
            .velocityFF(0)
            .maxMotion
                .maxVelocity(0)
                .maxAcceleration(0)
                .allowedClosedLoopError(0);

        config.smartCurrentLimit(60);

        sparkmax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        inputs.position = Rotations.of(encoder.getPosition());
        inputs.velocity = RPM.of(encoder.getVelocity());
        inputs.voltage = Volts.of(sparkmax.getBusVoltage());
        inputs.current = Amps.of(sparkmax.getOutputCurrent());
    }

    @Override
    public void runPosition(Angle position) {
        pid.setReference(position.in(Rotations), ControlType.kPosition);
    }

    @Override
    public void runVelocity(AngularVelocity velocity) {
        pid.setReference(velocity.in(RPM), ControlType.kVelocity);
    }

    @Override
    public void stop() {
        sparkmax.stopMotor();
    }
}
