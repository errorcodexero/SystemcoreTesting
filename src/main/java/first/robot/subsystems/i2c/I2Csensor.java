package first.robot.subsystems.i2c;

import org.wpilib.command2.SubsystemBase;

public class I2Csensor extends SubsystemBase {

   private final I2CsensorIO io;

    public I2Csensor(I2CsensorIO io){
      this.io = io;
       }
       
       @Override
       public void periodic() {

         //add inputs for the I2C color sensor 

       }

  }
