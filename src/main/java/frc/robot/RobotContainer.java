// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.OnboardIMU.MountOrientation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOSystemcore;
import frc.robot.subsystems.drive.ModuleIOReplay;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

public class RobotContainer {
  private Drive drive;

  private final CommandXboxController gamepad = new CommandXboxController(0);

  public RobotContainer() {

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPETITION:
          drive = new Drive(
            new GyroIOSystemcore(MountOrientation.kFlat), 
            ModuleIOTalonFX::new,
            CompTunerConstants.FrontLeft,
            CompTunerConstants.FrontRight,
            CompTunerConstants.BackLeft,
            CompTunerConstants.BackRight,
            CompTunerConstants.kCANBus,
            CompTunerConstants.kSpeedAt12Volts
          );

          break;
        case SIMBOT:
          drive = new Drive(
            new GyroIO() {}, 
            ModuleIOSim::new,
            CompTunerConstants.FrontLeft,
            CompTunerConstants.FrontRight,
            CompTunerConstants.BackLeft,
            CompTunerConstants.BackRight,
            CompTunerConstants.kCANBus,
            CompTunerConstants.kSpeedAt12Volts
          );

          break;
      }
    }

    if (drive == null) {
      drive = new Drive(
        new GyroIO() {}, 
        ModuleIOReplay::new,
        CompTunerConstants.FrontLeft,
        CompTunerConstants.FrontRight,
        CompTunerConstants.BackLeft,
        CompTunerConstants.BackRight,
        CompTunerConstants.kCANBus,
        CompTunerConstants.kSpeedAt12Volts
      );
    }

    DriveCommands.configure(drive, () -> -gamepad.getLeftY(), () -> -gamepad.getLeftX(), () -> -gamepad.getRightX());

    configureBindings();
    configureDriveBindings();
  }

  private void configureBindings() {

  };

  private void configureDriveBindings() {
    drive.setDefaultCommand(DriveCommands.joystickDrive());

    gamepad.leftBumper().whileTrue(DriveCommands.joystickDrive());
    gamepad.y().and(gamepad.b()).onTrue(drive.resetGyroCmd());
    gamepad.x().onTrue(drive.stopWithXCmd());

    gamepad.start().and(RobotModeTriggers.test()).toggleOnTrue(DriveCommands.wheelRadiusCharacterization(drive));
    gamepad.back().and(RobotModeTriggers.test()).toggleOnTrue(DriveCommands.feedforwardCharacterization(drive));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
