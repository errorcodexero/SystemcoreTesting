// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package first.robot.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import first.generated.CompTunerConstants;
import first.robot.Constants;
import first.robot.Constants.Mode;
import first.robot.util.LocalADStarAK;
import first.robot.util.LoggedTracer;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiFunction;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

public class Drive extends SubsystemBase {
  // These Constants should be the same for every drivebase, so just use the comp bot constants.
  static final double ODOMETRY_FREQUENCY = CompTunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public final double DRIVE_BASE_RADIUS;

  // Gyro degrees-per-rotation correction/trim
  static final double GYRO_YAW_DEG_PER_ROT_CORRECTION = -0.97;

  // These constants should change for every drivebase
  private final LinearVelocity SPEED_12_VOLTS;
  private final RobotConfig PP_CONFIG;

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 65.7709;
  private static final double ROBOT_MOI = 6.33;
  private static final double WHEEL_COF = 1.2;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private final Alert gyroDisconnectedAlert =
      new Alert(
          "gyroDisconnectedAlert", "Disconnected gyro, using kinematics as fallback.", Level.HIGH);

  private final SwerveDriveKinematics kinematics;
  private Rotation2d rawGyroRotation = Rotation2d.ZERO;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator;

  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      flConfig_;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frConfig_;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      blConfig_;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      brConfig_;

  public Drive(
      GyroIO gyroIO,
      BiFunction<
              SwerveModuleConstants<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>,
              CANBus,
              ModuleIO>
          moduleConstructor,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          flConfig,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          frConfig,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          blConfig,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          brConfig,
      CANBus canbus,
      LinearVelocity kSpeed12Volts) {
    // Setup Module IOs
    this.gyroIO = gyroIO;
    modules[0] = new Module(moduleConstructor.apply(flConfig, canbus), 0, flConfig);
    modules[1] = new Module(moduleConstructor.apply(frConfig, canbus), 1, frConfig);
    modules[2] = new Module(moduleConstructor.apply(blConfig, canbus), 2, blConfig);
    modules[3] = new Module(moduleConstructor.apply(brConfig, canbus), 3, brConfig);

    flConfig_ = flConfig;
    frConfig_ = frConfig;
    blConfig_ = blConfig;
    brConfig_ = brConfig;

    DRIVE_BASE_RADIUS =
        Math.max(
            Math.max(
                Math.hypot(flConfig.LocationX, flConfig.LocationY),
                Math.hypot(frConfig.LocationX, frConfig.LocationY)),
            Math.max(
                Math.hypot(blConfig.LocationX, blConfig.LocationY),
                Math.hypot(brConfig.LocationX, brConfig.LocationY)));

    // Pose Estimators and Kinematics
    kinematics = new SwerveDriveKinematics(getModuleTranslations());
    poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.ZERO);

    SPEED_12_VOLTS = kSpeed12Volts;

    PP_CONFIG =
        new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                flConfig.WheelRadius,
                SPEED_12_VOLTS.in(MetersPerSecond),
                WHEEL_COF,
                DCMotor.getKrakenX60Foc(1).withReduction(flConfig.DriveMotorGearRatio),
                flConfig.SlipCurrent,
                1),
            getModuleTranslations());

    // Usage reporting for swerve template
    // TODO: fix usage reporting
    // HAL.report(tResourceType.kResourceType_RobotDrive,
    // tInstances.kRobotDriveSwerve_AdvantageKit);
    // HAL.reportUsage(Resource, getName());

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // TODO: Configure AutoBuilder for PathPlanner
    // AutoBuilder.configure(
    //     this::getPose,
    //     this::setPose,
    //     this::getChassisVelocities,
    //     this::runVelocity,
    //     new PPHolonomicDriveController(
    //         new PIDConstants(8.0, 0.0, 0.0),           // 8 - original     13, 0.5
    //         new PIDConstants(8.0, 0.0, 0.0)),          // 8 - original
    //     PP_CONFIG,
    //     () -> MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED,
    //     this
    // );

    Pathfinding.setPathfinder(new LocalADStarAK());

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    LoggedTracer.reset();

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (RobotState.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (RobotState.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distance - lastModulePositions[moduleIndex].distance,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);

    LoggedTracer.record("DrivePeriodic");
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param velocities velocities in meters/sec
   */
  public void runVelocity(ChassisVelocities velocities) {
    // Calculate module setpoints
    ChassisVelocities discreteVelocities = velocities.discretize(0.02);
    SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(discreteVelocities);
    setpointStates =
        SwerveDriveKinematics.desaturateWheelVelocities(setpointStates, SPEED_12_VOLTS);

    // Log unoptimized setpoints and setpoint velocities
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisVelocities/Setpoints", discreteVelocities);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs the drive at the desired velocities.
   *
   * @param x X Velocity
   * @param y Y Velocity
   * @param omega Angular Velocity
   */
  public void runVelocity(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
    runVelocity(new ChassisVelocities(x, y, omega));
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle().orElseThrow();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // public void pointWheelsAt() {

  //   Rotation2d[] headings = new Rotation2d[4];
  //   for (int i = 0; i < 4; i++) {
  //     headings[i] =
  //   }
  // }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  public Command resetGyroCmd(Rotation2d rotation) {
    return runOnce(
            () -> {
              setPose(new Pose2d(getPose().getTranslation(), rotation));
            })
        .ignoringDisable(true);
  }

  public Command resetGyroCmd() {
    return resetGyroCmd(Rotation2d.ZERO);
  }

  public Command stopWithXCmd() {
    return runOnce(() -> stopWithX());
  }

  public Command runVelocityCmd(ChassisVelocities velocities) {
    return startEnd(
        () -> {
          runVelocity(velocities);
        },
        this::stop);
  }

  public Command runVelocityCmd(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
    return runVelocityCmd(new ChassisVelocities(x, y, omega));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleVelocity[] getModuleStates() {
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis velocities of the robot. */
  @AutoLogOutput(key = "SwerveChassisVelocities/Measured")
  public ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleStates());
  }

  @AutoLogOutput(key = "SwerveChassisVelocities/FieldRelativeMeasured")
  public ChassisVelocities getFieldChassisVelocities() {
    return getChassisVelocities().toRobotRelative(getRotation());
  }

  @AutoLogOutput(key = "SwerveChassisVelocities/TotalVelocity")
  public LinearVelocity getVelocity() {
    ChassisVelocities velocities = getChassisVelocities();
    double x = velocities.vx;
    double y = velocities.vy;

    return MetersPerSecond.of(Math.sqrt(x * x + y * y));
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Whether the rotation of the robot is near to a target. */
  public boolean rotationIsNear(Rotation2d target, Angle tolerance) {
    return target.getMeasure().isNear(getRotation().getMeasure(), tolerance);
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    Logger.recordOutput("Odometry/VisionMeasurement", visionRobotPoseMeters);
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return SPEED_12_VOLTS.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  public RobotConfig getPathplannerConfig() {
    return PP_CONFIG;
  }

  /** Returns an array of module translations. */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(flConfig_.LocationX, flConfig_.LocationY),
      new Translation2d(frConfig_.LocationX, frConfig_.LocationY),
      new Translation2d(blConfig_.LocationX, blConfig_.LocationY),
      new Translation2d(brConfig_.LocationX, brConfig_.LocationY)
    };
  }
}
