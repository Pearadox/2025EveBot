// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.groundCoralIntakeConstants;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.RobotContainer;

/* Define motors, encoders, and controllers
 * Create functions to raise/lower pivot
 * Make it so that lowering the pivot will also turn on intake, raising pivot will also turn off intake
 * Create a function to outtake 
 */

public class groundCoralIntake extends SubsystemBase {
  // Creates a new groundCoralIntake
  private PearadoxSparkMax armPivot;

  private RelativeEncoder armPivotEncoder;
  private SparkClosedLoopController armPivotPIDController;

  private double armPivotAdjust = 0.0;

  public enum ArmPivotPos {
    stowed, deployed
  }

  public ArmPivotPos armPivotPos = ArmPivotPos.stowed;


  private static final groundCoralIntake GROUND_CORAL_INTAKE = new  groundCoralIntake();

  // returns the created groundCoralIntake object
  public static groundCoralIntake getInstance() {
    return GROUND_CORAL_INTAKE;
  }

  // constructor - instantiates the motor, motor relative encoder, and motor PID (closedloop) controller
  public groundCoralIntake() {
    armPivot = new PearadoxSparkMax(groundCoralIntakeConstants.PIVOT_ID, MotorType.kBrushless, IdleMode.kBrake, 0, false,
    groundCoralIntakeConstants.PIVOT_kP, groundCoralIntakeConstants.PIVOT_kI, groundCoralIntakeConstants.PIVOT_kD,
    groundCoralIntakeConstants.PIVOT_MIN_OUTPUT, groundCoralIntakeConstants.PIVOT_MAX_OUTPUT);

    armPivotEncoder = armPivot.getEncoder();
    armPivotPIDController = armPivot.getClosedLoopController();

  }
  // sets armPivot mode to stowed
  public void setStowed() {
    armPivotPos = ArmPivotPos.stowed;
  }
  // sets armPivot mode to deployed
  public void setDeployed() {
    armPivotPos= ArmPivotPos.deployed;
  }

  @Override
  public void periodic() {
    // if it's stowed, it will jump to the stowed position - if it's deployed, it will jump to the deployed position
    if (armPivotPos == ArmPivotPos.stowed) {
      armPivotPIDController.setReference(groundCoralIntakeConstants.PIVOT_STOWED_ROT, ControlType.kPosition);
    }
    else if (armPivotPos == ArmPivotPos.deployed) {
      armPivotPIDController.setReference(groundCoralIntakeConstants.PIVOT_DEPLOYED_ROT, ControlType.kPosition);
    }

    // maps a toggle to the op controller left trigger - switches between deployed mode and stowed mode
    if (RobotContainer.opController.getLeftTriggerAxis() >= 0.95 && armPivotPos == ArmPivotPos.stowed) {
      setDeployed();
    }
    if (RobotContainer.opController.getLeftTriggerAxis() >= 0.95 && armPivotPos == ArmPivotPos.stowed) {
      setStowed();
    }
  }
}
