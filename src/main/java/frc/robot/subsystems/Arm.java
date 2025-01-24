// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DifferentialPositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private PearadoxTalonFX pivot;

  private ArmMode armMode = ArmMode.Intake;
  private enum ArmMode {
    Intake, L2, L3, L4
  }

  public static final Arm arm = new Arm();
  public static Arm getInstance() {
    return arm;
  }

  public Arm() {
    pivot = new PearadoxTalonFX(ArmConstants.ARM_KRAKEN_ID, NeutralModeValue.Brake, ArmConstants.CURRENT_LIMIT, true);
  }

  @Override
  public void periodic() {
    
  }

  public void ArmHold() {
    if(armMode == ArmMode.Intake) {
      pivot.setControl(new DifferentialPositionVoltage(ArmConstants.ARM_INTAKE_ROT, ArmConstants.ARM_INTAKE_ROT - getPivotPosition()));
      //TODO: create conversions so the above works correctly
    }
    else if(armMode == ArmMode.L2) {

    }
    else if(armMode == ArmMode.L3) {
      
    }
    else if(armMode == ArmMode.L4) {
      
    }
  }


  public void setArmIntake() {
    armMode = ArmMode.Intake;
  }
  public void setArmL2() {
    armMode = ArmMode.L2;
  }
  public void setArmL3() {
    armMode = ArmMode.L3;
  }
  public void setArmL4() {
    armMode = ArmMode.L4;
  }
  public double getPivotPosition() {
    return pivot.getPosition().getValueAsDouble();
  }
}
