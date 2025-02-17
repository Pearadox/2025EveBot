// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private PearadoxTalonFX elevator;
  private PearadoxTalonFX elevatorFollower;
  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private double elevatorOffset = 0.0;

  private static enum ElevatorMode { STOWED, LEVEL_TWO, LEVEL_THREE, LEVEL_FOUR; }
  private ElevatorMode elevatorMode = ElevatorMode.STOWED;
  
  private static Elevator ELEVATOR = new Elevator();
  public static Elevator getInstance() {
    return ELEVATOR;
  } 
  /** Creates a new Elevator. */
  public Elevator() {
    elevator = new PearadoxTalonFX(
             ElevatorConstants.ELEVATOR_ID,
             ElevatorConstants.MODE, 
             ElevatorConstants.CURRENT_LIMIT, 
             ElevatorConstants.IS_INVERTED);

   elevatorFollower = new PearadoxTalonFX(
             ElevatorConstants.ELEVATOR_FOLLOWER_ID,
             ElevatorConstants.MODE, 
             ElevatorConstants.CURRENT_LIMIT, 
             ElevatorConstants.IS_INVERTED);

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kG = ElevatorConstants.kG; // add enough Gravity Gain just before motor starts moving
    slot0Configs.kS = ElevatorConstants.kS; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = ElevatorConstants.kV; // A velocity target of 1 rps results in 0.1 V output
    slot0Configs.kA = ElevatorConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = ElevatorConstants.kP; // A position error of 2.5 rotations results in 12 V output, prev 4.8
    slot0Configs.kI = ElevatorConstants.kI; // no output for integrated error
    slot0Configs.kD = ElevatorConstants.kD; // A velocity error of 1 rps results in 0.1 V output

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MM_CRUISE_VELCOCITY; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MM_ACCELERATION; // Target acceleration of 160 rps/s (0.5 seconds)
    // (not sure if needed - > ) motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    elevator.getConfigurator().apply(talonFXConfigs);

    elevator.getPosition().setUpdateFrequency(ElevatorConstants.UPDATE_FREQ);
    elevator.getVelocity().setUpdateFrequency(ElevatorConstants.UPDATE_FREQ);

    // These are needed for the follower motor to work
    elevator.getDutyCycle().setUpdateFrequency(ElevatorConstants.UPDATE_FREQ);
    elevator.getMotorVoltage().setUpdateFrequency(ElevatorConstants.UPDATE_FREQ);
    elevator.getTorqueCurrent().setUpdateFrequency(ElevatorConstants.UPDATE_FREQ);
    elevator.optimizeBusUtilization();

    elevatorFollower.getConfigurator().apply(talonFXConfigs);
    elevatorFollower.optimizeBusUtilization();
    elevatorFollower.setControl(new Follower(ElevatorConstants.ELEVATOR_ID, false));

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position Inches", getElevatorPositionInches());
    SmartDashboard.putNumber("Elevator Velocity (in/sec)", getElevatorVelocity_InchesPerSecond());
    SmartDashboard.putNumber("Elevator Position Rots", getElevatorPositionRots());
    SmartDashboard.putNumber("Elevator Velocity (rot/sec)", getElevatorVelocity_RotsPerSecond());
  }

  public void setElevatorPosition() {

    if(elevatorMode == ElevatorMode.STOWED) {
      elevator.setControl(motionMagicRequest.withPosition(ElevatorConstants.STOWED_ROT + elevatorOffset));
    }
    else if(elevatorMode == ElevatorMode.LEVEL_TWO) {
      elevator.setControl(motionMagicRequest.withPosition(ElevatorConstants.LEVEL_TWO_ROT + elevatorOffset));
    }
    else if(elevatorMode == ElevatorMode.LEVEL_THREE) {
      elevator.setControl(motionMagicRequest.withPosition(ElevatorConstants.LEVEL_THREE_ROT + elevatorOffset));
    }
    else if(elevatorMode == ElevatorMode.LEVEL_FOUR) {
      elevator.setControl(motionMagicRequest.withPosition(ElevatorConstants.LEVEL_FOUR_ROT + elevatorOffset));
    }

  }

  public double getElevatorPositionRots() {
    return elevator.getPosition().getValueAsDouble();
  }

  public double getElevatorVelocity_RotsPerSecond() { 
    return elevator.getVelocity().getValueAsDouble();
  }

  public double getElevatorPositionInches() {
    return getElevatorPositionRots() * ElevatorConstants.kRotationToInches;
  }

  public double getElevatorVelocity_InchesPerSecond() {
    return getElevatorVelocity_RotsPerSecond() * ElevatorConstants.kRotationToInches;
  }

  public void changeElevatorOffset(double value) {
    elevatorOffset = MathUtil.clamp(elevatorOffset + value, 0, ElevatorConstants.MAX_ELEVATOR_ROT - ElevatorConstants.LEVEL_FOUR_ROT);
  }

  public void setElevatorStowedMode() {
    elevatorMode = ElevatorMode.STOWED;
  }

  public void setElevatorLevelTwoMode() {
    elevatorMode = ElevatorMode.LEVEL_TWO;
  }

  public void setElevatorLevelThreeMode() {
    elevatorMode = ElevatorMode.LEVEL_THREE;
  }

  public void setElevatorLevelFourMode() {
    elevatorMode = ElevatorMode.LEVEL_FOUR;
  }


}