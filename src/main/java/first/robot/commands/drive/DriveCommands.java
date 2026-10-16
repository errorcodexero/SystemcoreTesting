// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot.commands.drive;

import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import first.robot.Constants;
import first.robot.Constants.FieldConstants;
import first.robot.Constants.Mode;
import first.robot.subsystems.drive.Drive;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;

public class DriveCommands {
  private static final double kStoppedVelocity = 0.15;

  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 4.0;
  private static final double ANGLE_KD = 0.0;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 40.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private static Drive drive_;
  private static DoubleSupplier xSupplier_;
  private static DoubleSupplier ySupplier_;
  private static DoubleSupplier omegaSupplier_;
  private static boolean configured = false;

  private DriveCommands() {}

  /**
   * Configures the drive commands. In order to call convenience drive commands, this must be
   * configured beforehand.
   *
   * @param drive Drive subsystem
   * @param xSupplier Supplier of X velocity (negative left joystick Y)
   * @param ySupplier Supplier of Y velocity (negative left joystick X)
   * @param omegaSupplier Supplier of rotational velocity (negative right joystick X)
   */
  public static void configure(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    drive_ = drive;
    xSupplier_ = xSupplier;
    ySupplier_ = ySupplier;
    omegaSupplier_ = omegaSupplier;

    configured = true;
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.ZERO, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.ZERO))
        .getTranslation();
  }

  public static Pose2d rotateIfRed(Pose2d pose) {
    if (MatchState.getAlliance().isPresent() && MatchState.getAlliance().get() == Alliance.RED) {
      Translation2d center =
          new Translation2d(
              FieldConstants.field.getFieldLength() / 2.0,
              FieldConstants.field.getFieldWidth() / 2.0);
      pose = pose.rotateAround(center, Rotation2d.fromDegrees(180.0));
    }

    return pose;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   * This is preconfigured with {@link #configure(Drive, DoubleSupplier, DoubleSupplier,
   * DoubleSupplier)}
   */
  public static Command joystickDrive() {
    if (!configured)
      throw new IllegalStateException(
          "DriveCommands joystickDrive called without first configuring!");

    return joystickDrive(drive_, xSupplier_, ySupplier_, omegaSupplier_);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick. This is preconfigured with {@link #configure(Drive,
   * DoubleSupplier, DoubleSupplier, DoubleSupplier)}
   */
  public static Command joystickDriveAtAngle(Supplier<Rotation2d> rotationSupplier) {
    if (!configured)
      throw new IllegalStateException(
          "DriveCommands joystickDriveAtAngle called without first configuring!");

    return joystickDriveAtAngle(drive_, xSupplier_, ySupplier_, rotationSupplier);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return drive.runEnd(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisVelocities speeds =
              new ChassisVelocities(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              MatchState.getAlliance().isPresent()
                  && MatchState.getAlliance().get() == Alliance.RED;
          drive.runVelocity(
              speeds.toRobotRelative(
                  isFlipped ? drive.getRotation().plus(Rotation2d.PI) : drive.getRotation()));
        },
        drive::stop);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return drive
        .runEnd(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisVelocities speeds =
                  new ChassisVelocities(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  MatchState.getAlliance().isPresent()
                      && MatchState.getAlliance().get() == Alliance.RED;
              drive.runVelocity(
                  speeds.toRobotRelative(
                      isFlipped ? drive.getRotation().plus(Rotation2d.PI) : drive.getRotation()));
            },
            drive::stop)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisVelocities(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.ZERO;
    double gyroDelta = 0.0;
  }

  /**
   * Creates an on-the-fly path, and creates a command that follows that path. This is different to
   * pathfinding because it is ignorant of field obstacles, and should not be used when doing any
   * complex navigations.
   *
   * <p>Otherwise, when there are no obstacles between your current pose, and your target pose,
   * (e.g. 2025 reef targeting), this is a marginal performance increase.
   *
   * @param targetPose The pose to create an on-the-fly path to.
   * @return A command that follows the created path.
   */
  public static Command simplePathCommand(
      Drive drive, Pose2d targetPose, LinearVelocity v, LinearAcceleration a) {

    // Create Constraints
    PathConstraints constraints =
        new PathConstraints(
            v.in(MetersPerSecond),
            a.in(MetersPerSecondPerSecond),
            DegreesPerSecond.of(540).in(RadiansPerSecond),
            DegreesPerSecondPerSecond.of(720).in(RadiansPerSecondPerSecond));

    // Create Command Only When It Is Starting
    return Commands.defer(
        () -> {
          ChassisVelocities speed = drive.getFieldChassisVelocities();
          double vel = Math.hypot(speed.vx, speed.vy);
          Rotation2d heading = new Rotation2d(speed.vx, speed.vy);

          Pose2d curPose = drive.getPose();
          Transform2d curToTarget = targetPose.minus(curPose);

          if (vel < kStoppedVelocity) {
            heading =
                curPose.getRotation().plus(curToTarget.getTranslation().getAngle().orElseThrow());
          } else {
            heading = new Rotation2d(speed.vx, speed.vy);
          }

          Pose2d startWaypoint = new Pose2d(curPose.getTranslation(), heading);
          Pose2d endWaypoint = targetPose;

          if (Constants.getMode() != Mode.REAL) {
            Logger.recordOutput("SimplePathing/StartWaypoint", startWaypoint);
            Logger.recordOutput("SimplePathing/EndWaypoint", endWaypoint);
          }

          List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startWaypoint, endWaypoint);

          //
          // The robot is currently moving in a given direction.  The path needs to take into
          // account
          // this starting condition.
          //
          IdealStartingState start = new IdealStartingState(vel, drive.getPose().getRotation());

          PathPlannerPath path =
              new PathPlannerPath(
                  waypoints, constraints, start, new GoalEndState(0.0, targetPose.getRotation()));

          path.preventFlipping = true;

          // If the pose is less than 1 centimeter away, dont do anything. (This is
          // because of an error I am looking into)
          if (curPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.01) {
            return Commands.none();
          }

          return AutoBuilder.followPath(path);
        },
        Set.of(drive));
  }

  public static Command simplePathCommand(
      Drive drive, Pose2d targetPose, Pose2d immd, LinearVelocity v, LinearAcceleration a) {

    // Create Constraints
    PathConstraints constraints =
        new PathConstraints(
            v.in(MetersPerSecond),
            a.in(MetersPerSecondPerSecond),
            DegreesPerSecond.of(540).in(RadiansPerSecond),
            DegreesPerSecondPerSecond.of(720).in(RadiansPerSecondPerSecond));

    // Create Command Only When It Is Starting
    return Commands.defer(
        () -> {
          ChassisVelocities speed = drive.getFieldChassisVelocities();
          double vel = Math.hypot(speed.vx, speed.vy);
          Rotation2d heading = new Rotation2d(speed.vx, speed.vy);

          Pose2d curPose = drive.getPose();
          Transform2d curToTarget = targetPose.minus(curPose);

          if (vel < kStoppedVelocity) {
            heading =
                curPose.getRotation().plus(curToTarget.getTranslation().getAngle().orElseThrow());
          } else {
            heading = new Rotation2d(speed.vx, speed.vy);
          }

          Pose2d startWaypoint = new Pose2d(curPose.getTranslation(), heading);
          Pose2d endWaypoint = targetPose;

          if (Constants.getMode() != Mode.REAL) {
            Logger.recordOutput("SimplePathing/StartWaypoint", startWaypoint);
            Logger.recordOutput("SimplePathing/EndWaypoint", endWaypoint);
          }

          List<Waypoint> waypoints =
              PathPlannerPath.waypointsFromPoses(startWaypoint, immd, endWaypoint);

          //
          // The robot is currently moving in a given direction.  The path needs to take into
          // account
          // this starting condition.
          //
          IdealStartingState start = new IdealStartingState(vel, drive.getPose().getRotation());

          PathPlannerPath path =
              new PathPlannerPath(
                  waypoints, constraints, start, new GoalEndState(0.0, targetPose.getRotation()));

          path.preventFlipping = true;

          // If the pose is less than 1 centimeter away, dont do anything. (This is
          // because of an error I am looking into)
          if (curPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.01) {
            return Commands.none();
          }

          return AutoBuilder.followPath(path);
        },
        Set.of(drive));
  }

  /**
   * Creates a path based on Pathfinding. This will allow to avoid field elements, but at a
   * performance cost. TANSTAAFL
   *
   * @param targetPose
   * @return A command to follow that path.
   */
  public static Command swerveDriveToCommand(Pose2d targetPose) {
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfind =
        AutoBuilder.pathfindToPose(
            targetPose, constraints, MetersPerSecond.zero() // Goal end velocity in meters/sec
            );

    return pathfind;
  }

  /**
   * Creates a command to follow a path specified by a PathPlanner file, not setting the initial
   * pose. If this is the initial path of an auto mode, call {@link #initialFollowPathCmd()}
   * instead.
   *
   * @param pathName The name of the PathPlanner file to load.
   * @param mirroredX Whether to mirror the path across the x axis.
   * @return A Command that, when executed, will follow the specified path. If the path has a
   *     problem being created, a Command that does nothing.
   */
  public static Command followPathCommand(String pathName, boolean mirroredX) {
    Optional<PathPlannerPath> path = findPath(pathName, mirroredX);

    if (path.isPresent()) {
      return AutoBuilder.followPath(path.get());
    }

    return Commands.none();
  }

  /**
   * Creates a command to follow a path specified by a PathPlanner file, not setting the initial
   * pose. If this is the initial path of an auto mode, call {@link #initialFollowPathCmd()}
   * instead.
   *
   * @param pathName The name of the PathPlanner file to load.
   * @return A Command that, when executed, will follow the specified path. If the path has a
   *     problem being created, a Command that does nothing.
   */
  public static Command followPathCommand(String pathName) {
    return followPathCommand(pathName, false);
  }

  /**
   * Creates a command to follow a path specified by a PathPlanner file, setting the initial pose.
   *
   * @param pathName The name of the PathPlanner file to load.
   * @param mirroredX Whether to mirror the path across the x axis.
   * @param drive The drivebase to set the pose on.
   * @return A Command that, when executed, will follow the specified path. If the path has a
   *     problem being created, or it is zero in length, returns a Command that does nothing.
   */
  public static Command initialFollowPathCommand(Drive drive, String pathName, boolean mirroredX) {
    Optional<PathPlannerPath> path = findPath(pathName, mirroredX);

    return Commands.defer(
        () -> {
          if (path.isPresent()) {
            PathPlannerPath initPosePath = path.get();
            Alliance alliance = MatchState.getAlliance().orElse(Alliance.BLUE);

            if (alliance == Alliance.RED) {
              initPosePath = path.get().flipPath();
            }

            if (initPosePath.getStartingHolonomicPose().isEmpty()) {
              return Commands.none();
            }

            var startingPose = initPosePath.getStartingHolonomicPose().orElseThrow();

            return Commands.sequence(
                setPoseCommand(drive, startingPose, false), AutoBuilder.followPath(path.get()));
          }

          return Commands.none();
        },
        Set.of(drive));
  }

  /**
   * Creates a command to follow a path specified by a PathPlanner file, setting the initial pose.
   *
   * @param pathName The name of the PathPlanner file to load.
   * @param drive The drivebase to set the pose on.
   * @return A Command that, when executed, will follow the specified path. If the path has a
   *     problem being created, or it is zero in length, returns a Command that does nothing.
   */
  public static Command initialFollowPathCommand(Drive drive, String pathName) {
    return initialFollowPathCommand(drive, pathName, false);
  }

  /**
   * Sets the pose of the drivebase in a command.
   *
   * @param drive The drive subsystem.
   * @param pose The pose to set it to.
   * @return A command that sets the pose of the drivebase.
   */
  public static Command setPoseCommand(Drive drive, Pose2d pose, boolean perAlliance) {
    Optional<Alliance> alliance = MatchState.getAlliance();
    if (alliance.isPresent() && perAlliance && alliance.get() == Alliance.RED) {
      pose = rotateIfRed(pose);
    }

    Pose2d copy = pose;
    return Commands.runOnce(() -> drive.setPose(copy));
  }

  /**
   * Finds a pathplanner path.
   *
   * @param pathName The name of the path to find.
   * @param mirroredX Whether to mirror the path across the x axis.
   * @return The path, or an empty optional if it could not be found.
   */
  public static Optional<PathPlannerPath> findPath(String pathName, boolean mirroredX) {
    try {

      PathPlannerPath path =
          mirroredX
              ? PathPlannerPath.fromPathFile(pathName).mirrorPath()
              : PathPlannerPath.fromPathFile(pathName);

      return Optional.of(path);

    } catch (FileNotFoundException e) {
      System.err.println("Path file " + pathName + " not found!");
    } catch (IOException e) {
      System.err.println("Path file " + pathName + " could not be read!");
    } catch (ParseException e) {
      System.err.println("Path format in " + pathName + " is incorrect!");
    }

    return Optional.empty();
  }
}
