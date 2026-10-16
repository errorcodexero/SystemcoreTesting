// Copyright (c) 2026 FRC 1425 Error Code Xero 
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/**
 * Blank Replay Implementation That Does Nothing.
 *
 * <p>This exists because the constructor of the ModuleIO must fit a certain signature.
 */
public class ModuleIOReplay implements ModuleIO {
  public ModuleIOReplay(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      CANBus canbus) {}
}
