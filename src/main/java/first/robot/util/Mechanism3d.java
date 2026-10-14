package frc.robot.util;

import static org.wpilib.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;

public class Mechanism3d extends SubsystemBase {
  private static final Pose3d hoodZero = new Pose3d(-0.098, 0, 0.457, Rotation3d.ZERO);
  private static final Pose3d intakeZero = new Pose3d(0.252, 0, 0.207, Rotation3d.ZERO);

  private final String name;

  // Current Positions
  private Angle hood = Degrees.zero();
  private Angle intake = Degrees.zero();
  private Angle hopper = Degrees.zero();

  // Previous Positions
  private Angle previousIntake = Degrees.zero();

  // These are the points where the intake will catch onto the hopper shields.
  private Angle hopperUpperCatch = Degrees.zero();
  private Angle hopperLowerCatch = Degrees.of(35);

  private Mechanism3d(String name) {
    this.name = name;
    periodic();
  }

  public void setHood(Angle angle) {
    hood = angle;
  }

  public void setIntake(Angle angle) {
    intake = angle;
  }

  public void zero() {
    hood = Degrees.zero();
    intake = Degrees.zero();
    hopper = Degrees.zero();
    previousIntake = Degrees.zero();
  }

  private Pose3d pitchUp(Pose3d pose, Angle pitch) {
    return pose.transformBy(
        new Transform3d(Translation3d.ZERO, new Rotation3d(Degrees.zero(), pitch, Degrees.zero())));
  }

  public void periodic() {
    LoggedTracer.reset();

    Angle deltaIntake = intake.minus(previousIntake);

    // If outside the catch, add this intake delta to the hopper.
    if (intake.gt(hopperLowerCatch) || intake.lt(hopperUpperCatch)) {
      hopper = hopper.plus(deltaIntake);
      hopperLowerCatch = hopperLowerCatch.plus(deltaIntake);
      hopperUpperCatch = hopperUpperCatch.plus(deltaIntake);
      Logger.recordOutput("Mechanism3d/" + name + "/Catching", true);
    } else {
      Logger.recordOutput("Mechanism3d/" + name + "/Catching", false);
    }

    previousIntake = intake;

    Logger.recordOutput(
        "Mechanism3d/" + name,
        new Pose3d[] {
          pitchUp(hoodZero, hood), pitchUp(intakeZero, intake), pitchUp(intakeZero, hopper)
        });

    LoggedTracer.record("Mechanism3d");
  }

  public static final Mechanism3d measured = new Mechanism3d("Measured");
  public static final Mechanism3d setpoints = new Mechanism3d("Setpoints");
}
