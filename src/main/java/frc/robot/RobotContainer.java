// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ArmHold;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.ElevatorHold;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.util.vision.PoseEstimation;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final Elevator elevator = Elevator.getInstance();
    public static final Arm arm = Arm.getInstance();
    public static final EndEffector endEffector = EndEffector.getInstance();
    public static final Climber climbSubsystem = new Climber();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotOrientedDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05)
            .withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle pointTowards = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controller
    public static final XboxController driverController = new XboxController(0);
    public static final XboxController opController = new XboxController(1);

    // driver controls
    private final JoystickButton resetHeading_Start =
            new JoystickButton(driverController, XboxController.Button.kStart.value);
    private final POVButton alignPovLeft = new POVButton(driverController, 270);
    private final POVButton alignPovDown = new POVButton(driverController, 180);
    private final POVButton alignPovRight = new POVButton(driverController, 90);
    private final JoystickButton zeroElevator_LB =
            new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton robotOrientated_RB =
            new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    private final POVButton driveForwardPovUp = new POVButton(driverController, 0);

    // operator controls
    private final JoystickButton levelFour_Y = new JoystickButton(opController, XboxController.Button.kY.value);
    private final JoystickButton levelThree_B = new JoystickButton(opController, XboxController.Button.kB.value);
    private final JoystickButton levelTwo_X = new JoystickButton(opController, XboxController.Button.kX.value);
    private final JoystickButton stow_A = new JoystickButton(opController, XboxController.Button.kA.value);
    private final JoystickButton station_Start = new JoystickButton(opController, XboxController.Button.kStart.value);
    private final JoystickButton barge_Back = new JoystickButton(opController, XboxController.Button.kBack.value);
    private final JoystickButton coralAlgaeSwap_LB =
            new JoystickButton(opController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton resetAdjusts_RB =
            new JoystickButton(opController, XboxController.Button.kRightBumper.value);
    private final POVButton elevatorAdjustUp_POV_0 = new POVButton(opController, 0);
    private final POVButton elevatorAdjustDown_POV_180 = new POVButton(opController, 180);
    private final POVButton armAdjustUp_POV_90 = new POVButton(opController, 90);
    private final POVButton armAdjustDown_POV_270 = new POVButton(opController, 270);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final SendableChooser<Command> autoChooser;

    private final AutoAlign align;
    //     private final Command alignLeftBranch;
    //     private final Command alignRightBranch;
    //     private final Command alignMid;

    public static final PoseEstimation poseEstimation = new PoseEstimation();

    public RobotContainer() {
        setDefaultCommands();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("GH_L4");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();

        align = new AutoAlign(() -> drivetrain.getState().Pose);
        // alignLeftBranch = getReefAlignCommand(AlignConstants.REEF_ALIGN_LEFT_TX);
        // alignRightBranch = getReefAlignCommand(AlignConstants.REEF_ALIGN_RIGHT_TX);
        // alignMid = getReefAlignCommand(AlignConstants.REEF_ALIGN_MID_TX);
    }

    private void configureBindings() {
        robotOrientated_RB.whileTrue(drivetrain.applyRequest(
                () -> robotOrientedDrive
                        .withVelocityX(drivetrain.frontLimiter.calculate(-driverController.getLeftY())
                                * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(drivetrain.sideLimiter.calculate(-driverController.getLeftX())
                                * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(drivetrain.turnLimiter.calculate(-driverController.getRightX())
                                * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        driveForwardPovUp.onTrue(drivetrain
                .applyRequest(() -> robotOrientedDrive
                        .withVelocityX(-MaxSpeed / 2)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                .withTimeout(0.2));

        zeroElevator_LB
                .whileTrue(new InstantCommand(() -> elevator.setZeroing(true)))
                .onFalse(new InstantCommand(() -> elevator.zeroElevator())
                        .andThen(new InstantCommand(() -> elevator.setZeroing(false))));

        alignPovDown.whileTrue(drivetrain.applyRequest(
                () -> robotOrientedDrive
                        .withVelocityX(align.getAlignForwardSpeedPercent2(AlignConstants.REEF_ALIGN_TZ)
                                * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(align.getAlignStrafeSpeedPercent2(AlignConstants.REEF_ALIGN_MID_TX)
                                * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(align.getAlignRotationSpeedPercent(align.getAlignAngleReef())
                                * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        alignPovLeft.whileTrue(drivetrain.applyRequest(
                () -> robotOrientedDrive
                        .withVelocityX(align.getAlignForwardSpeedPercent2(AlignConstants.REEF_ALIGN_TZ) * MaxSpeed)
                        .withVelocityY(align.getAlignStrafeSpeedPercent2(AlignConstants.REEF_ALIGN_LEFT_TX) * MaxSpeed)
                        .withRotationalRate(align.getAlignRotationSpeedPercent(align.getAlignAngleReef())
                                * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        alignPovRight.whileTrue(drivetrain.applyRequest(() -> robotOrientedDrive
                .withVelocityX(align.getAlignForwardSpeedPercent2(AlignConstants.REEF_ALIGN_TZ) * MaxSpeed)
                .withVelocityY(align.getAlignStrafeSpeedPercent2(AlignConstants.REEF_ALIGN_RIGHT_TX) * MaxSpeed)
                .withRotationalRate(align.getAlignRotationSpeedPercent(align.getAlignAngleReef()) * MaxAngularRate)));

        // strafeTriggers.whileTrue(
        //     drivetrain.applyRequest(
        //         () -> robotOrientedDrive
        //             .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y
        // (forward)
        //             .withVelocityY(
        //                     (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis())
        //                     * 0.5 * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise
        // with negative X (left)
        //     )
        // );

        resetHeading_Start.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        stow_A.onTrue(new InstantCommand(() -> elevator.setElevatorStowedMode())
                .andThen(new InstantCommand(() -> arm.setStowed())));
        station_Start.onTrue(new InstantCommand(() -> elevator.setElevatorStationMode())
                .andThen(new WaitCommand(0.5))
                .andThen(new InstantCommand(() -> arm.setArmIntake())));
        levelTwo_X.onTrue(new InstantCommand(() -> elevator.setElevatorLevelTwoMode())
                .andThen(new InstantCommand(() -> arm.setArmL2())));
        levelThree_B.onTrue(new InstantCommand(() -> elevator.setElevatorLevelThreeMode())
                .andThen(new InstantCommand(() -> arm.setArmL3())));
        levelFour_Y.onTrue(new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                .andThen(new InstantCommand(() -> arm.setArmL4())));
        barge_Back.onTrue(new InstantCommand(() -> elevator.setElevatorBarge())
                .andThen(new InstantCommand(() -> arm.setBarge())));

        resetAdjusts_RB.onTrue(
                new InstantCommand(() -> elevator.resetAdjust()).andThen(new InstantCommand(() -> arm.resetAdjust())));

        // climb_RB.onTrue(new InstantCommand(() -> arm.setClimb())
        //         .andThen(new InstantCommand(() -> elevator.setElevatorStowedMode())));

        coralAlgaeSwap_LB.onTrue(new InstantCommand(() -> elevator.changeIsCoral())
                .andThen(new InstantCommand(() -> arm.changeIsCoral())));

        elevatorAdjustUp_POV_0.whileTrue(
                new RunCommand(() -> elevator.changeElevatorOffset(ElevatorConstants.ELEVATOR_OFFSET)));
        elevatorAdjustDown_POV_180.whileTrue(
                new RunCommand(() -> elevator.changeElevatorOffset(-ElevatorConstants.ELEVATOR_OFFSET)));
        armAdjustUp_POV_90.whileTrue(new RunCommand(() -> arm.armAdjust(ArmConstants.ARM_ADJUST_INCREMENT)));
        armAdjustDown_POV_270.whileTrue(new RunCommand(() -> arm.armAdjust(-ArmConstants.ARM_ADJUST_INCREMENT)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public void registerNamedCommands() {
        NamedCommands.registerCommand(
                "LevelStation",
                new InstantCommand(() -> elevator.setElevatorStationMode())
                        .andThen(new InstantCommand(() -> arm.setArmIntake())));
        NamedCommands.registerCommand("ElevatorStowedMode", new InstantCommand(() -> elevator.setElevatorStowedMode()));
        NamedCommands.registerCommand(
                "LevelTwo",
                new InstantCommand(() -> elevator.setElevatorLevelTwoMode())
                        .andThen(new InstantCommand(() -> arm.setArmL2())));
        NamedCommands.registerCommand(
                "LevelThree",
                new InstantCommand(() -> elevator.setElevatorLevelThreeMode())
                        .andThen(new InstantCommand(() -> arm.setArmL3())));
        NamedCommands.registerCommand(
                "LevelFour",
                new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                        .andThen(new InstantCommand(() -> arm.setArmL4())));

        NamedCommands.registerCommand("Outtake", new InstantCommand(() -> endEffector.coralIn()));
        NamedCommands.registerCommand("Intake", new InstantCommand(() -> endEffector.coralOut()));
        NamedCommands.registerCommand("Stop EE", new InstantCommand(() -> endEffector.stopCoral()));
    }

    public void setDefaultCommands() {
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> fieldCentricDrive
                                .withVelocityX(drivetrain.frontLimiter.calculate(-driverController.getLeftY())
                                        * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(drivetrain.sideLimiter.calculate(-driverController.getLeftX())
                                        * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(drivetrain.turnLimiter.calculate(-driverController.getRightX())
                                        * MaxAngularRate) // Drive counterclockwise with negative X (left)
                        ));

        elevator.setDefaultCommand(new ElevatorHold());
        arm.setDefaultCommand(new ArmHold());
    }
}
