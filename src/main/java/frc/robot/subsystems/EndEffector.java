// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;
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

  private PearadoxTalonFX endEffector;
  private DigitalInput endSensor;
  private Debouncer debouncer;

  private boolean rumbled = false;
  private boolean isExtended = false; //TODO: integrate with arm
  private boolean isHolding = false;


  public EndEffector() {
    endEffector = new PearadoxTalonFX(EndEffectorConstants.END_EFFECTOR_ID, NeutralModeValue.Brake, 50, false);
    endSensor = new DigitalInput(EndEffectorConstants.END_SENSOR_CHANNEL);
    debouncer = new Debouncer(0.2, DebounceType.kFalling);

    BaseStatusSignal.setUpdateFrequencyForAll(ArmConstants.UPDATE_FREQ, 
        endEffector.getPosition(), 
        endEffector.getVelocity(),
        endEffector.getDutyCycle(),
        endEffector.getMotorVoltage(),
        endEffector.getTorqueCurrent(),
        endEffector.getSupplyCurrent(),
        endEffector.getStatorCurrent()
    );
  }

  @Override
  public void periodic() {
    collectCoral();

    SmartDashboard.putBoolean("End Sensor", hasCoral());

    SmartDashboard.putNumber("EE Stator Current", endEffector.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("EE Supply Current", endEffector.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("EE Volts", endEffector.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("EE Angular Velocity", endEffector.getVelocity().getValueAsDouble());
  }

  public void collectCoral() {
    if(RobotContainer.driverController.getRightTriggerAxis() >= 0.9){
      coralIn();
    } else if(RobotContainer.driverController.getLeftTriggerAxis() >= 0.9){
      coralOut();
    }else{
      slowEndEffector();
    }
  }

  public void coralIn(){
    endEffector.set(EndEffectorConstants.PULL_VOLTAGE * RobotContainer.driverController.getRightTriggerAxis());
  }

  public void coralOut(){
    endEffector.set(EndEffectorConstants.PUSH_VOLTAGE * RobotContainer.driverController.getLeftTriggerAxis());
  }

  public void holdCoral(){
    endEffector.set(-0.05);
  }

  public void slowEndEffector(){
    endEffector.set(0.1);
  }
  public void stopEndEffector(){
    endEffector.set(0);
  }

  public boolean hasCoral(){
    return endEffector.getStatorCurrent().getValueAsDouble() > 15;
    // return debouncer.calculate(!endSensor.get());
  }

  public boolean getHolding(){
    return isHolding;
  }

  public void setHolding(boolean hold){
    isHolding = hold;
  }
}
