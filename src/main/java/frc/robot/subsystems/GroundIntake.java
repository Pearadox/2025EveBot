// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.IntakeConstants;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */
  private PearadoxTalonFX pivot;

  PositionVoltage pivotRequest = new PositionVoltage(0).withSlot(0);
  
  private double pivotAdjust = 0.0;
  
  public enum PivotPos {
    stowed, intake, outtake, algae
  }

  public PivotPos pivotPos = PivotPos.stowed;
  
  private  PearadoxTalonFX roller;

  private static final GroundIntake GROUND_INTAKE = new GroundIntake();
  
  public static GroundIntake getInstance() {
    return GROUND_INTAKE;
  }
  
  public GroundIntake() {
    pivot = new PearadoxTalonFX(
      IntakeConstants.PIVOT_ID,
      IntakeConstants.MODE,
      IntakeConstants.PIVOT_CURRENT_LIMIT,
      false);

    Slot0Configs slot0configs = new Slot0Configs();

    slot0configs.kP = IntakeConstants.PIVOT_kP;
    slot0configs.kI = IntakeConstants.PIVOT_kI;
    slot0configs.kD = IntakeConstants.PIVOT_kD;

    pivot.getConfigurator().apply(slot0configs);

    roller = new PearadoxTalonFX(
      IntakeConstants.ROLLER_ID,
      IntakeConstants.MODE,
      IntakeConstants.ROLLER_CURRENT_LIMIT,
      false);
    
  }

  public void setStowed() {
    pivotPos = PivotPos.stowed;
  }

  public void setIntake() {
    pivotPos = PivotPos.intake;
  }
  
  public void setOuttake() {
    pivotPos = PivotPos.outtake;
  }

  public void setAlgae() {
    pivotPos = PivotPos.algae;
  }

  public void changePivotActivePos() {
    if (pivotPos == PivotPos.intake) {
      pivotPos = PivotPos.outtake;
    }
    else if (pivotPos == PivotPos.outtake) {
      pivotPos = PivotPos.intake;
    }
  }

  public PivotPos getPivotPos() {
    return pivotPos;
  }

  public void pivotHold() {
    if (pivotPos == PivotPos.stowed) {
      pivot.setControl(pivotRequest.withPosition(IntakeConstants.PIVOT_STOWED_ROT));
      roller.set(1.0);
    }
    else if (pivotPos == PivotPos.intake) {
      pivot.setControl(pivotRequest.withPosition(IntakeConstants.PIVOT_INTAKE_ROT));
      roller.set(-1.0);
    }
    else if (pivotPos == PivotPos.outtake) {
      pivot.setControl(pivotRequest.withPosition(IntakeConstants.PIVOT_OUTTAKE_ROT));
      roller.set(0.0);
    }
    else if (pivotPos == PivotPos.algae) {
      pivot.setControl(pivotRequest.withPosition(IntakeConstants.PIVOT_ALGAE_ROT));
      roller.set(-1.0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Ground Intake Supply Cuurent", pivot.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Ground Intake Stator Cuurent", pivot.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Ground Intake position", pivot.getPosition().getValueAsDouble());
  }
}