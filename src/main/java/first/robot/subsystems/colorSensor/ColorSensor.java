// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.colorSensor;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.I2C;

import com.revrobotics.ColorSensorV3;

import first.robot.subsystems.motor.MotorInputsAutoLogged;

@SuppressWarnings("unused")
public class ColorSensor extends SubsystemBase {
    final ColorSensorV3 ColorSensorPort1 = new ColorSensorV3(I2C.Port.PORT_1);

    private final ColorSensorIO io;
    private ColorSensorIO.ColorInputs inputs = new ColorSensorIO.ColorInputs();
    

    public ColorSensor(ColorSensorIO io) {
        this.io = io;
    }



    @Override
    public void periodic() {
        io.updateColorInputs1(inputs, ColorSensorPort1);
}
}