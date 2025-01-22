// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.GroundIntake.ArmPivotPos;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundIntakeHold extends Command {
  /** Creates a new groundIntakeHold. */

  GroundIntake groundIntake = GroundIntake.getInstance();

  public GroundIntakeHold() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(groundIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    groundIntake.groundIntakeHold();

    // sets the mode to stowed/deployed on a toggle mapped to op controller left bumper
    if (RobotContainer.opController.getLeftTriggerAxis() >= 0.95 && groundIntake.getPivotPos() == ArmPivotPos.stowed) {
      groundIntake.setDeployed();
    }
    else if (RobotContainer.opController.getLeftTriggerAxis() >= 0.95 && groundIntake.getPivotPos() == ArmPivotPos.deployed) {
      groundIntake.setStowed();
    }

    // outtakes rollers if op controller left bumper pressed, else intakes/outtakes based on stowed/deployed
    if (!RobotContainer.opController.getLeftBumperButton()) {
      if (groundIntake.getPivotPos() == ArmPivotPos.stowed) {
        groundIntake.rollerStop();
      }
      else {
        groundIntake.rollerIntake();
      }
    }
    else {
      groundIntake.rollerOuttake();
    }
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
