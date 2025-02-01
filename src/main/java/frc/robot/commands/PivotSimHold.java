// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.groundintake.GroundIntake.ArmPivotPos;
import frc.robot.subsystems.groundintake.GroundIntakeSim;
import frc.robot.subsystems.groundintake.GroundIntakeSim.SimPivotMode;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotSimHold extends Command {

  GroundIntakeSim groundIntakeSim = GroundIntakeSim.getInstance();

  /** Creates a new PivotSimHold. */
  public PivotSimHold() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(groundIntakeSim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    groundIntakeSim.simIntakeHold();

    if (RobotContainer.opController.getLeftTriggerAxis() >= 0.95 && groundIntakeSim.getSimPivotPos() == SimPivotMode.stowed) {
      groundIntakeSim.setDeployed();
    }
    else if (RobotContainer.opController.getLeftTriggerAxis() >= 0.95 && groundIntakeSim.getSimPivotPos() == SimPivotMode.deployed) {
      groundIntakeSim.setStowed();
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
