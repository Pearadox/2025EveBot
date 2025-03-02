// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ClimbConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;


/** Class to run the rollers over CAN */
public class ClimbSubsystem extends SubsystemBase {
  //private final SparkMax climbMotor;
  private final PearadoxTalonFX climbMotor = new PearadoxTalonFX(ClimbConstants.CLIMB_MOTOR_ID, NeutralModeValue.Brake, 50, false);
  public ClimbSubsystem() {
        
    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    //climbMotor.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    //climbConfig.voltageCompensation(ClimbConstants.CLIMB_MOTOR_VOLTAGE_COMP);
  }

  @Override
  public void periodic() {
  }

  /** This is a method that makes the roller spin */
  public void runClimb(double forward, double reverse) {
    climbMotor.set(forward - reverse);
  }
}
