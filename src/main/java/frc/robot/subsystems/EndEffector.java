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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffector extends SubsystemBase {

  private static final EndEffector END_EFFECTOR = new EndEffector();

  public static EndEffector getInstance(){
    return END_EFFECTOR;
  }

  private PearadoxSparkMax endEffector;
  private DigitalInput endSensor;
  private Debouncer debouncer;

  private boolean rumbled = false;
  private boolean isExtended = false; //TODO: integrate with arm

  public EndEffector() {
    endEffector = new PearadoxSparkMax(EndEffectorConstants.END_EFFECTOR_ID, MotorType.kBrushless, IdleMode.kBrake, 50, false);
    endSensor = new DigitalInput(EndEffectorConstants.END_SENSOR_CHANNEL);
    debouncer = new Debouncer(0.2, DebounceType.kFalling);
  }

  @Override
  public void periodic() {
    collectCoral();

    SmartDashboard.putBoolean("End Sensor", hasCoral());
  }

  public void collectCoral() {
    if(RobotContainer.driverController.getRightTriggerAxis() >= 0.95){
      coralIn();
    } else if(RobotContainer.driverController.getLeftTriggerAxis() >= 0.95){
      coralOut();
    } else if(hasCoral()){
      stopEndEffector();
    } else{
      stopEndEffector();
    }
  }

  public void coralIn(){
    endEffector.set(EndEffectorConstants.PULL_VOLTAGE);
  }

  public void coralOut(){
    endEffector.set(-EndEffectorConstants.PUSH_VOLTAGE);
  }

  public void stopEndEffector(){
    endEffector.set(0.0);
  }

  public boolean hasCoral(){
    return debouncer.calculate(!endSensor.get());
  }
}
