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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.ElevatorHold;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = Drivetrain.getInstance();

  public static final Elevator elevator = Elevator.getInstance();
  public static final Arm arm = Arm.getInstance();
  public static final EndEffector endEffector = EndEffector.getInstance();

  //Driver Controls
  public static final XboxController driverController = new XboxController(IOConstants.DRIVER_CONTROLLER_PORT);

  private final JoystickButton resetHeading_Start = new JoystickButton(driverController, XboxController.Button.kStart.value);
  private final JoystickButton elevatorUp = new JoystickButton(driverController, XboxController.Button.kY.value);
  private final JoystickButton elevatorDown = new JoystickButton(driverController, XboxController.Button.kA.value);
  private final JoystickButton armAdjustUp = new JoystickButton(driverController, XboxController.Button.kX.value);
  private final JoystickButton armAdjustDown = new JoystickButton(driverController, XboxController.Button.kB.value);

  private final JoystickButton elevatorStowed = new JoystickButton(opController, XboxController.Button.kX.value);
  private final JoystickButton elevatorLevelTwo = new JoystickButton(opController, XboxController.Button.kY.value);
  private final JoystickButton elevatorLevelThree = new JoystickButton(opController, XboxController.Button.kB.value);
  private final JoystickButton elevatorLevelFour = new JoystickButton(opController, XboxController.Button.kA.value);

  //Operator Controls
  public static final XboxController opController = new XboxController(IOConstants.OP_CONTROLLER_PORT);  

  //Pose Estimation

  //Shuffleboard
  public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. 
   * @throws IOException */
  public RobotContainer() throws IOException {
    registerNamedCommands();
    configureBindings();
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
  private void configureBindings() {
    //Driver Buttons
    resetHeading_Start.onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));

    //TODO: uncomment
    // elevatorUp.whileTrue(new RunCommand(() -> elevator.changeElevatorOffset(ElevatorConstants.ELEVATOR_OFFSET)));
    // elevatorDown.whileTrue(new RunCommand(() -> elevator.changeElevatorOffset(-ElevatorConstants.ELEVATOR_OFFSET)));
    // armAdjustUp.whileTrue(new RunCommand(() -> arm.armAdjust(ArmConstants.ARM_ADJUST_INCREMENT)));
    // armAdjustDown.whileTrue(new RunCommand(() -> arm.armAdjust(-ArmConstants.ARM_ADJUST_INCREMENT)));
    // elevatorStowed.onTrue(new InstantCommand(() -> elevator.setElevatorStowedMode()));
    // elevatorLevelTwo.onTrue(new InstantCommand(() -> elevator.setElevatorLevelTwoMode()));
    // elevatorLevelThree.onTrue(new InstantCommand(() -> elevator.setElevatorLevelThreeMode()));
    // elevatorLevelFour.onTrue(new InstantCommand(() -> elevator.setElevatorLevelFourMode()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands(){
    NamedCommands.registerCommand("Stop Modules", new InstantCommand(() -> drivetrain.stopModules()));
    NamedCommands.registerCommand("Reset Heading", new InstantCommand(drivetrain::zeroHeading, drivetrain));
  }

  public void setDefaultCommands(){
    drivetrain.setDefaultCommand(new SwerveDrive());
    elevator.setDefaultCommand(new ElevatorHold());
  }
}