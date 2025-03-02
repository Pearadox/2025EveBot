// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.config.ServoChannelConfig.PulseRange;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class IOConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OP_CONTROLLER_PORT = 1;
  }

  public static final class SwerveConstants{
    //Drivetrain motor/encoder IDs
    public static final int LEFT_FRONT_DRIVE_ID = 1;
    public static final int RIGHT_FRONT_DRIVE_ID = 2;
    public static final int LEFT_BACK_DRIVE_ID = 3;
    public static final int RIGHT_BACK_DRIVE_ID = 4;
    
    public static final int LEFT_FRONT_TURN_ID = 5;
    public static final int RIGHT_FRONT_TURN_ID = 6;
    public static final int LEFT_BACK_TURN_ID = 7;
    public static final int RIGHT_BACK_TURN_ID = 8;
    
    public static final int LEFT_FRONT_CANCODER_ID = 11;
    public static final int RIGHT_FRONT_CANCODER_ID = 12;
    public static final int LEFT_BACK_CANCODER_ID = 13;
    public static final int RIGHT_BACK_CANCODER_ID = 14;

    public static final int PIGEON_ID = 15;

    //Drivetrain characteristics
    public static final double LEFT_FRONT_OFFSET = -0.197998; //1
    public static final double RIGHT_FRONT_OFFSET = 0.041748; //2
    public static final double LEFT_BACK_OFFSET = -0.253174; //3
    public static final double RIGHT_BACK_OFFSET = -0.341064;//4

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 5.357; //SDS Mk4i L3+ (60:16 first stage)
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;
    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION;
    public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
    public static final double KP_TURNING = 0.5;

    public static final double DRIVETRAIN_MAX_SPEED = 6.62;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 3.45 * Math.PI; //TODO: Determine max angular speed

    //Swerve Kinematics
    public static final double TRACK_WIDTH = Units.inchesToMeters(24.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(22.75);
    public static final double DRIVE_BASE_RADIUS = Math.sqrt(Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2)) / 2.0;

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    //Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.25;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 3;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 3;
    public static final double ROBOT_ORIENTED_TRIGGER_OFFSET = 0.4;

    //Auton constraints
    public static final double AUTO_kP_TRANSLATION = 4;
    public static final double AUTO_kP_ROTATION = 1.5;


    public static final double AUTO_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1.5;
    public static final double AUTO_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 2.0;
    public static final double AUTO_DRIVE_MAX_ACCELERATION = 3;
    public static final double AUTO_DRIVE_MAX_ANGULAR_ACCELERATION = Math.PI;

    //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html
    public static final Vector<N3> ODOMETRY_STD_DEV = VecBuilder.fill(0.1, 0.1, 0.1);

    public static final double kS_PERCENT = 0.035;
    public static final double kP_PERCENT = 0.009;

    //exponent on joystick sensitivity 
    public static final int EXPONENT = 3;
  }

  public static final class EndEffectorConstants{
    public static final int END_EFFECTOR_ID = 23;

    public static final double PULL_SPEED = -1;
    public static final double PUSH_SPEED = 0.5;


    public static final int END_SENSOR_CHANNEL = 0;
  }

  public static final class FieldConstants{
    public static final double FIELD_LENGTH = 16.54175;
    public static final double FIELD_WIDTH = 8.21055;

    public static final double SPEAKER_HEIGHT = Units.inchesToMeters(80.515);
  }

  public static final class VisionConstants{
    //Limelight Offest with Mount
    // LL Forward = 0.2667 LL Up = 0.46355
    public static final String LL_NAME = "limelight";

    public static final Transform3d ROBOT_TO_SHOOTER_LL = new Transform3d(
      new Translation3d( 
        -Units.inchesToMeters(0.2732),
        Units.inchesToMeters(5.8752),
        Units.inchesToMeters(26.4144)
      ), 
      new Rotation3d());

      // public static final PhotonVisionBackend.StandardDeviation PHOTON_VISION_STD_DEV =
      // (distance, count) -> {
      //     double distanceMultiplier = Math.pow(distance - ((count - 1) * 2), 2);
      //     double translationalStdDev = (0.05 / (count)) * distanceMultiplier + 0.05;
      //     double rotationalStdDev = 0.2 * distanceMultiplier + 0.1;
      //     return VecBuilder.fill(
      //             translationalStdDev,
      //             translationalStdDev,
      //             rotationalStdDev
      //     );
      // };

      public static final Vector<N3> LIMELIGHT_STD_DEV = VecBuilder.fill(.7, .7, .9999999);
      
      public static final double AMBIGUITY_FILTER = 0.3;
      public static final double DISTANCE_FILTER = FieldConstants.FIELD_LENGTH / 2;
  }

  public static final class ElevatorConstants {

    public static final int ELEVATOR_ID = 21;
    public static final int ELEVATOR_FOLLOWER_ID = 20;
    public static final NeutralModeValue MODE  = NeutralModeValue.Brake;
    public static final int CURRENT_LIMIT = 60; //TODO
    public static final boolean IS_INVERTED = false;

    public static final int UPDATE_FREQ = 50;

    public static final double MAX_VELOCITY_MPS = 2.0; //TODO
    public static final double MAX_ACCELERATION_MPS2 = 8.0; //TODO
    public static final double MM_CRUISE_VELCOCITY = 45; //TODO
    public static final double MM_ACCELERATION = 20; //TODO

    public static final double TICKS_PER_REV = 4000; //TODO
    public static final double GEAR_RATIO = 3; //TODO
    public static final double PULLEY_DIAMETER = 2.005; //TODO should be chain pitch * number of teeth / pi
    public static final double kRotationToInches = PULLEY_DIAMETER * Math.PI / GEAR_RATIO;

    //the following are in inches
    public static final double STOWED_HEIGHT = 0;
    public static final double STATION_HEIGHT = 1.3; //TODO
    public static final double LEVEL_TWO_HEIGHT = 9.1; // was 7 This is slightly away from the reef for clearance //TODO
    public static final double LEVEL_THREE_HEIGHT = 25; //TODO was 21[]\

    public static final double LEVEL_FOUR_HEIGHT = 30; //TODO


    public static final double MAX_ELEVATOR_HEIGHT = 20; //TODO

    public static final double LEVEL_TWO_ROT = LEVEL_TWO_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
    public static final double STATION_ROT = STATION_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
    public static final double LEVEL_THREE_ROT = LEVEL_THREE_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
    public static final double LEVEL_FOUR_ROT = LEVEL_FOUR_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
    public static final double MAX_ELEVATOR_ROT = MAX_ELEVATOR_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
    public static final double STOWED_ROT = STOWED_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);


    // public static final double STATION_ROT = 3.868;
    // public static final double LEVEL_TWO_ROT = 3.868;
    // public static final double LEVEL_THREE_ROT = 10;
    // public static final double LEVEL_FOUR_ROT = 15.0;
    // public static final double MAX_ELEVATOR_ROT = 15.5; // 15.65
    // public static final double STOWED_ROT = 0;

    public static final double ELEVATOR_OFFSET = 0.05;

    //TODO: change all of these values to match true elevator gains
    public static final double kG = 0.245; // tuned according to wpilib docs; may need to lower
    public static final double kS = 0.; //0.2
    public static final double kV = 0.; //0.3
    public static final double kA = 0.0; //0.02
    public static final double kP = 0.6;//0.4
    public static final double kI = 0.00; //0
    public static final double kD = 0.000; //0
  }

  public static final class ArmConstants {
    //TODO: Update all constants for arm    
    public static final int ARM_KRAKEN_ID = 22;
    public static final int CURRENT_LIMIT = 40;

    public static final double ARM_GEAR_RATIO = 60; // TODO?

    public static final double ARM_LEVEL_4_ROT = Units.degreesToRotations(189.2529297); //-174.7470703125
    public static final double ARM_LEVEL_3_ROT = Units.degreesToRotations(-81.19); //was -74.455078125
    public static final double ARM_LEVEL_2_ROT = Units.degreesToRotations(-81.19); // was -74.455078125
    public static final double ARM_INTAKE_ROT = Units.degreesToRotations(50.04589843750001);    //  was 61.....
    public static final double ARM_STOWED_ROT = Units.degreesToRotations(-88); //should be 0

    public static final double ARM_ADJUST_INCREMENT = 0.075;

    public static final double UPDATE_FREQ = 50;

    // TODO: tune pid
    public static final double kG = 0.2;
    public static final double kS = 0.0;
    public static final double kV = 0.1;
    public static final double kA = 0.0;
    public static final double kP = 0.2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
  
  public static final class ClimbConstants {
    public static final int CLIMB_MOTOR_ID = 24;
    public static final int CLIMB_MOTOR_CURRENT_LIMIT = 40;
    public static final double CLIMB_MOTOR_VOLTAGE_COMP = 10;
    public static final double CLIMB_VALUE = 0.7;
  }

  public static final class IntakeConstants {
    // CANID for pivot
    public static final int PIVOT_ID = 40; // real CANID

    // public static final int PIVOT_GEAR_RATIO = 60;
    
    // PID for pivot
    public static final double PIVOT_kP = 0.5;
    public static final double PIVOT_kI = 0.0;
    public static final double PIVOT_kD = 0.0;
    
    public static final double PIVOT_MIN_OUTPUT = -0.5;
    public static final double PIVOT_MAX_OUTPUT = 0.5;
    
    public static final double PIVOT_GEARING = 60; // 25:1 reduction gear ratio

    public static final double PIVOT_OUTTAKE_ROT = Units.degreesToRotations(10.0) * PIVOT_GEARING; // TODO: find outtake rot in motor rotations
    public static final double PIVOT_INTAKE_ROT = Units.degreesToRotations(105.0) * PIVOT_GEARING; // TODO: find intake rot in motor rotations
    public static final double PIVOT_STOWED_ROT = Units.degreesToRotations(0.0) * PIVOT_GEARING; // TODO: find stowed rot in motor rotations
    public static final double PIVOT_ALGAE_ROT = Units.degreesToRotations(40.0) * PIVOT_GEARING; // TODO: find stowed rot in motor rotations
    
    // CAN ID for roller
    public static final int ROLLER_ID = 41;
  }
}