// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;

public class EndEffector extends SubsystemBase {

  private static final EndEffector END_EFFECTOR = new EndEffector();

  private PearadoxSparkMax endEffector;
  private DigitalInput endSensor;
  private Debouncer debouncer;

  private boolean hasCoral = false;
  private boolean rumbled = false;
  private boolean isExtended = false; //TODO: integrate with arm

  /** Creates a new EndEffector. */
  public EndEffector() {
    endEffector = new PearadoxSparkMax(EndEffectorConstants.END_EFFECTOR_ID, MotorType.kBrushless, IdleMode.kBrake, 50, false);
    endSensor = new DigitalInput(EndEffectorConstants.END_SENSOR_CHANNEL);
    debouncer = new Debouncer(0.2, DebounceType.kFalling);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    collectCoral();
  }

  public void collectCoral() {
    if(RobotContainer.driverController.getRightTriggerAxis() >= 0.95){
      endEffector.set(EndEffectorConstants.PULL_VOLTAGE);
    } else if(RobotContainer.driverController.getLeftTriggerAxis() >= 0.95){
      endEffector.set(-EndEffectorConstants.PULL_VOLTAGE);
    } else if(hasCoral()){
      endEffector.set(0);
    } else {
      endEffector.set(0);
    }
  }
  
  public static EndEffector getInstance(){
    return END_EFFECTOR;
  }

  public boolean hasCoral(){
    return debouncer.calculate(!endSensor.get());
  }
}
