package frc.robot.subsystems.drive;

import java.util.Queue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.OnboardIMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.OnboardIMU.MountOrientation;

/**
 * A gyro implementation for the onboard IMU included in the Systemcore controller.
 */
public class GyroIOSystemcore implements GyroIO {

    private final OnboardIMU imu;

    private final Queue<Double> timestamps;
    private final Queue<Double> yawPositions;
    
    public GyroIOSystemcore(MountOrientation orientation) {
        imu = new OnboardIMU(orientation);
        imu.resetYaw();
        yawPositions = PhoenixOdometryThread.getInstance().registerSignal(imu::getYawRadians);
        timestamps = PhoenixOdometryThread.getInstance().registerSignal(Timer::getFPGATimestamp);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;

        // Instant values
        inputs.yawPosition = imu.getRotation2d();
        inputs.yawVelocityRadPerSec = imu.getGyroRateZ();
        
        // Read from the thread
        Drive.odometryLock.lock();

        inputs.odometryYawPositions = yawPositions.stream().map(rads -> Rotation2d.fromRadians(rads)).toArray(Rotation2d[]::new);
        inputs.odometryYawTimestamps = timestamps.stream().mapToDouble(time -> time).toArray();

        yawPositions.clear();
        timestamps.clear();

        Drive.odometryLock.unlock();
    }
    
}
