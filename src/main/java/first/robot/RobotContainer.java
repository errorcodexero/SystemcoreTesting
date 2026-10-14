// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.*;

import first.generated.CompTunerConstants;
import first.robot.Constants.DriveConstants;
import first.robot.commands.drive.DriveCommands;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.GyroIOPigeon2;
import first.robot.subsystems.drive.ModuleIOReplay;
import first.robot.subsystems.drive.ModuleIOTalonFX;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandXboxController;

public class RobotContainer {

  private Drive drive;

  private final CommandXboxController gamepad_ = new CommandXboxController(0);

  public RobotContainer() {
    switch (Constants.getRobot()) {
      case COMPETITION:
        drive =
            new Drive(
                new GyroIOPigeon2(
                    CompTunerConstants.DrivetrainConstants.Pigeon2Id, CompTunerConstants.kCANBus),
                ModuleIOTalonFX::new,
                CompTunerConstants.FrontLeft,
                CompTunerConstants.FrontRight,
                CompTunerConstants.BackLeft,
                CompTunerConstants.BackRight,
                CompTunerConstants.kCANBus,
                CompTunerConstants.kSpeedAt12Volts);
        break;
      case SIMBOT:
        break;
      default:
        break;
    }

    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              ModuleIOReplay::new,
              CompTunerConstants.FrontLeft,
              CompTunerConstants.FrontRight,
              CompTunerConstants.BackLeft,
              CompTunerConstants.BackRight,
              CompTunerConstants.kCANBus,
              CompTunerConstants.kSpeedAt12Volts);
    }

    DriveCommands.configure(
        drive, () -> -gamepad_.getLeftY(), () -> -gamepad_.getLeftX(), () -> -gamepad_.getRightX());

    configureBindings();
    configureDriveBindings();
  }

  // Bind robot actions to commands here.
  private void configureBindings() {}

  private void configureDriveBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(DriveCommands.joystickDrive().withName("JoystickDrive"));

    // Slow Mode, during left bumper
    gamepad_
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -gamepad_.getLeftY() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad_.getLeftX() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad_.getRightX() * DriveConstants.slowModeJoystickMultiplier));

    // Switch to X pattern / brake while X button is pressed
    gamepad_.x().whileTrue(drive.stopWithXCmd());

    // Robot Relative
    gamepad_
        .dpadUp()
        .whileTrue(
            drive.runVelocityCmd(
                FeetPerSecond.one(), MetersPerSecond.of(0), RadiansPerSecond.zero()));

    gamepad_
        .dpadDown()
        .whileTrue(
            drive.runVelocityCmd(
                FeetPerSecond.one().unaryMinus(), MetersPerSecond.of(0), RadiansPerSecond.zero()));

    gamepad_
        .dpadLeft()
        .whileTrue(
            drive.runVelocityCmd(
                MetersPerSecond.zero(), FeetPerSecond.one(), RadiansPerSecond.zero()));

    gamepad_
        .dpadRight()
        .whileTrue(
            drive.runVelocityCmd(
                MetersPerSecond.zero(), FeetPerSecond.one().unaryMinus(), RadiansPerSecond.zero()));

    // Reset gyro to 0° when Y & B button is pressed
    gamepad_.y().and(gamepad_.b()).onTrue(drive.resetGyroCmd());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
