// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot;

import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.framework.RobotBase;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  /** CONFIGURATION */

  // Sets the currently running robot. Change to SIMBOT when running the
  // desktop physics simulation so AdvantageKit runs in SIM mode instead of
  // falling back to REPLAY.
  private static final RobotType robotType = RobotType.SIMBOT;

  public static class DriveConstants {
    public static final double slowModeJoystickMultiplier = 0.4;
  }

  public static class FieldConstants {
    public static final Field field = Fields.FRC_2026_REBUILT_WELDED.loadField();
  }

  /** ROBOT STATE */
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum RobotType {
    /** The competition bot */
    COMPETITION,

    /** The Sim Bot */
    SIMBOT
  }

  // This is only a fallback! This will not change the robot type.
  private static final RobotType defaultRobotType = RobotType.COMPETITION;

  private static final Alert invalidRobotType =
      new Alert(
          "invalidRobotType",
          "Invalid RobotType selected. Defaulting to " + defaultRobotType.toString(),
          Level.MEDIUM);

  public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      invalidRobotType.set(true);
      return defaultRobotType;
    }

    return robotType;
  }

  public static final Mode getMode() {
    return switch (getRobot()) {
      case SIMBOT -> Mode.SIM;
      default -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    };
  }
}
