// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberUp extends Command {
  
  private Climber climber = Climber.getInstance();

  public ClimberUp() {
    addRequirements(climber);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    climber.runClimber(ClimberConstants.CLIMBER_SPEED_UP);
  }

  
  @Override
  public void end(boolean interrupted) {
    climber.runClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
