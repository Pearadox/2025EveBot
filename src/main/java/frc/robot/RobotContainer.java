// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.IOConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;

import frc.robot.commands.PivotHold;
import frc.robot.commands.PivotSimHold;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.groundintake.GroundIntakeSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = Drivetrain.getInstance();

  public static final GroundIntake groundIntake = GroundIntake.getInstance();
  public static final GroundIntakeSim simIntake = GroundIntakeSim.getInstance();

  //Driver Controls
  public static final XboxController driverController = new XboxController(IOConstants.DRIVER_CONTROLLER_PORT);

  private final JoystickButton resetHeading_Start = new JoystickButton(driverController, XboxController.Button.kStart.value);
  
  //Operator Controls
  public static final CommandXboxController opController = new CommandXboxController(IOConstants.OP_CONTROLLER_PORT);
  

  //Pose Estimation

  //Shuffleboard
  public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. 
   * @throws IOException */
  public RobotContainer() throws IOException {
    registerNamedCommands();
    configureButtonBindings();
    setDefaultCommands();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link[\]
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    //Driver Buttons
    resetHeading_Start.onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));

    opController.b().onTrue(Commands.runOnce(() -> simIntake.setDeployed(), simIntake));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drivetrain.resetAllEncoders();
    if(drivetrain.isRedAlliance()){
      drivetrain.setHeading(60);
    }
    else{
      drivetrain.setHeading(-60);
    }
    return autoChooser.getSelected();
  }

  public void registerNamedCommands(){
    NamedCommands.registerCommand("Stop Modules", new InstantCommand(() -> drivetrain.stopModules()));
    NamedCommands.registerCommand("Reset Heading", new InstantCommand(drivetrain::zeroHeading, drivetrain));
  }

  public void setDefaultCommands(){
    drivetrain.setDefaultCommand(new SwerveDrive());
    groundIntake.setDefaultCommand(new PivotHold());
    simIntake.setDefaultCommand(new PivotSimHold());
    
  }


}