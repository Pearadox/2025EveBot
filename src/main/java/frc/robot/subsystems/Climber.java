// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {


  private PearadoxTalonFX climber;
  private static final Climber CLIMBER = new Climber();
  public static final Climber getInstance() {
    return CLIMBER;
  }


  public Climber() {
    
    climber = new PearadoxTalonFX(
            ClimberConstants.CLIMBER_ID, 
            NeutralModeValue.Brake, 
            ClimberConstants.CLIMBER_CURRENT_LIMIT, 
            ClimberConstants.CLIMBER_IS_INVERTED);

  }

  public void periodic() {
    //TODO: add smartdashboard data (?)
  }

  public void runClimber(double speed) {
    climber.set(speed);
  }
}
