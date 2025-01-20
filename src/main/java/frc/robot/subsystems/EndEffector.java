// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
  /** Creates a new EndEffector. */
  public EndEffector() {
    //endEffector = new PearadoxSparkMax(EndEffectorConstants.END_EFFECTOR_ID, brushed, null, 0, false)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
