package first.robot.subsystems.i2c;

import org.wpilib.command2.SubsystemBase;

public interface I2CsensorIO {
 
 public static class I2CsensorIOInputs{
    public boolean isConnected = false; 
    public double redValue= 0.0;
    public double greenValue = 0.0;
    public double blueValue = 0.0;
    public int proximity = 0;
 }


}
