// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.chrono.IsoChronology;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorHold extends Command {

  EndEffector ee = EndEffector.getInstance();
  /** Creates a new EndEffectorHold. */
  public EndEffectorHold() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ee);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(RobotContainer.driverController.getRightTriggerAxis() >= 0.95 && !ee.hasCoral()){
    //   ee.coralIn();
    // }else if(RobotContainer.driverController.getLeftTriggerAxis() >= 0.95 && !ee.hasCoral()){
    //  ee.coralOut(); 
    // }else if(ee.hasCoral()){
    //   ee.holdCoral();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
