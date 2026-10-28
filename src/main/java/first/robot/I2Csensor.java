package first.robot;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.hardware.bus.I2C;
import org.wpilib.hardware.bus.I2C.Port;
import org.wpilib.util.Color;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

public class I2Csensor extends SubsystemBase {

  private static final I2C.Port I2C_PORT = (PORT_1).kOnboard;

  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C_PORT);
  private final ColorMatch colorMatcher = new ColorMatch();

  public IntakeColorSubsystem()
  {
    
  }

  // Example color used to define the targets : used blue
    private final Color kbluetarget = new Color();
    private final Color KGreentarget = new Color();
    private final Color kRedtarget = new Color();
    return getColor()

  }
}