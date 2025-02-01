// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundintake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot; // how and why do we use this?
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.GroundIntakeConstants;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.RobotContainer;

/* Define motors, encoders, and controllers
 * Create functions to raise/lower pivot - DONE
 * Make it so that lowering the pivot will also turn on intake, raising pivot will also turn off intake - DONE
 * Create a function to outtake that overrides current roller function at the given position - DONE
 * implement beam-break sensor to turn off intake w/o having to raise pivot
 */

public class GroundIntake extends SubsystemBase {
  // Declaring objects for the PIVOT
  private PearadoxSparkMax armPivot;

  private RelativeEncoder armPivotEncoder;
  private SparkClosedLoopController armPivotPIDController;

  private double armPivotAdjust = 0.0;

  public enum ArmPivotPos {
    stowed, deployed
  }

  public ArmPivotPos armPivotPos = ArmPivotPos.stowed;

  // Declaring objects for the ROLLERS
  private PearadoxSparkMax armRoller;

  private DigitalInput irSensor;

  // creates the ONLY groundCoralIntake object to be used
  private static final GroundIntake GROUND_CORAL_INTAKE = new GroundIntake();

  // returns the created groundCoralIntake object
  public static GroundIntake getInstance() {
    return GROUND_CORAL_INTAKE;
  }

  // constructor - instantiates the motor, motor relative encoder, and motor PID (closedloop) controller
  // also instantiates the roller motor and the IR sensor
  public GroundIntake() {
    // Initializing objects for the PIVOT
    armPivot = new PearadoxSparkMax(GroundIntakeConstants.PIVOT_ID, MotorType.kBrushless, IdleMode.kBrake, 0, false,
                GroundIntakeConstants.PIVOT_kP, GroundIntakeConstants.PIVOT_kI, GroundIntakeConstants.PIVOT_kD,
                GroundIntakeConstants.PIVOT_MIN_OUTPUT, GroundIntakeConstants.PIVOT_MAX_OUTPUT); // TODO: Find motor limit and inversion

    armPivotEncoder = armPivot.getEncoder();
    armPivotPIDController = armPivot.getClosedLoopController();

    // Initializing objects for the ROLLER
    armRoller = new PearadoxSparkMax(GroundIntakeConstants.ROLLER_ID, MotorType.kBrushless, IdleMode.kBrake, 0, false);// TODO: find motor limit and inversion

    irSensor = new DigitalInput(GroundIntakeConstants.IR_SENSOR_CHANNEL);

  }
  // updating/getting the PIVOT's positions
  public void setStowed() {
    armPivotPos = ArmPivotPos.stowed;
  }
  
  public void setDeployed() {
    armPivotPos = ArmPivotPos.deployed;
  }

  public ArmPivotPos getPivotPos() {
    return armPivotPos;
  }

  // updating the ROLLER's speed
  public void rollerIntake() {
    armRoller.set(0.5);
  }

  public void rollerStop() {
    armRoller.set(0.0);
  }

  public void rollerOuttake() {
    armRoller.set(-0.5);
  }
  
  // the actual command that's going to run
  public void groundIntakeHold() {
    // if it's stowed, it will jump to the stowed position - if it's deployed, it will jump to the deployed position
    if (armPivotPos == ArmPivotPos.stowed) {
      armPivotPIDController.setReference(GroundIntakeConstants.PIVOT_STOWED_ROT, ControlType.kPosition);
    }
    else if (armPivotPos == ArmPivotPos.deployed) {
      armPivotPIDController.setReference(GroundIntakeConstants.PIVOT_DEPLOYED_ROT, ControlType.kPosition);
    }
  }

  @Override
  public void periodic() {
    /* moved to groundIntakeHold.java
    * // maps a toggle to the op controller left trigger - switches between deployed mode and stowed mode
    * if (RobotContainer.opController.getLeftTriggerAxis() >= 0.95 && armPivotPos == ArmPivotPos.stowed) {
    *   setDeployed();
    *   rollerIntake();
    * }
    * if (RobotContainer.opController.getLeftTriggerAxis() >= 0.95 && armPivotPos == ArmPivotPos.stowed) {
    *   setStowed();
    *   rollerStop();
    * }

    * if (RobotContainer.opController.getLeftBumper()) {
    *   armRoller.set(-0.5);
    * }
    * else if (armPivotPos == ArmPivotPos.stowed) {
    *   armRoller.set(0.0);
    * }
    * else if (armPivotPos == ArmPivotPos.deployed) {
    *   armRoller.set(0.5);
    * }
    */
  }
}
