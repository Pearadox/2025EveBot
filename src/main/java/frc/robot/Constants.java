package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static class VisionConstants {
        public static final String LL_NAME = "limelight-back";
        public static final String LL_B_NAME = "limelight-coral-station"; // TODO: limelight names & offsets

        public static final Vector<N3> LIMELIGHT_STD_DEV = VecBuilder.fill(.7, .7, .9999999);
        public static final Vector<N3> MEGATAG2_LIMELIGHT_STD_DEV = VecBuilder.fill(.7, .7, .9999999);

        public static final double AMBIGUITY_FILTER = 0.3;
        public static final double DISTANCE_FILTER = FieldConstants.FIELD_LENGTH / 2;

        // // : determine questnav std dev (likely lower than this)
        // public static final Vector<N3> QUESTNAV_STD_DEV = VecBuilder.fill(.7, .7, .9999999);

        // TODO: use andymark field layout for fit events
        public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    public static final class FieldConstants {
        public static final double FIELD_LENGTH = Units.inchesToMeters(690.876);
        public static final double FIELD_WIDTH = Units.inchesToMeters(317);

        public static final int[] BLUE_REEF_TAG_IDS = {18, 19, 20, 21, 17, 22};
        public static final int[] BLUE_CORAL_STATION_TAG_IDS = {12, 13};
        public static final int[] RED_REEF_TAG_IDS = {7, 6, 11, 10, 9, 8};
        public static final int[] RED_CORAL_STATION_TAG_IDS = {1, 2};
    }

    public static final class DriveConstants {
        public static final double ROBOT_ORIENTED_TRIGGER_OFFSET = 0.4;

        public static final double DEFAULT_DEADBAND = 0.07;
        public static final double DRIVER_ALIGNING_DEADBAND = 0.15;
    }

    public static final class AlignConstants {
        // TODO: possible align pid adjustment
        public static final double ALIGN_STRAFE_KP = 0.04;
        public static final double ALIGN_STRAFE_KI = 0.001;
        public static final double ALIGN_FORWARD_KP = 0.05; // -0.06
        public static final double ALIGN_KS = 0.009;

        // tx and ty tolerances with setpoint
        public static final double ALIGN_TOLERANCE_PIXELS = 0.5;
        // don't try translationally aligning unless rotation is already aligned within this tolerance
        public static final double ALIGN_ROT_TOLERANCE_DEGREES = 10;

        // reduce speed by 1/4 every tick when an april tag is not seen
        public static final double ALIGN_DAMPING_FACTOR = 0.75;
        public static final double ALIGN_SPEED_DEADBAND = 0.025;

        public static final double BRANCH_SPACING = Units.inchesToMeters(12.94 / 2.0);

        // target relative
        public static final double REEF_ALIGN_MID_TX = 0; // 0.28575
        public static final double REEF_ALIGN_LEFT_TX = -BRANCH_SPACING + REEF_ALIGN_MID_TX - 0.05;
        public static final double REEF_ALIGN_RIGHT_TX = BRANCH_SPACING + REEF_ALIGN_MID_TX + 0.04 - 0.05;
        public static final double REEF_ALIGN_TZ = -0; // target relative

        public static final double STATION_ALIGN_TX = 0.05;
        public static final double STATION_ALIGN_TZ = 0.5;

        public static final double REEF_kP = 0.5; // Tune all PID values
        public static final double REEF_kI = 0;
        public static final double REEF_kD = 0;

        public static final double REEF_Forward_kP = 0.2; // Tune all PID values

        public static final double ROT_REEF_kP = 0.02; // Tune all PID values
        public static final double ROT_REEF_kI = 0;
        public static final double ROT_REEF_kD = 0;

        public static final PathConstraints PATH_CONSTRAINTS =
                new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    }

    public static final class ElevatorConstants {

        public static final int ELEVATOR_ID = 21;
        public static final int ELEVATOR_FOLLOWER_ID = 20;
        public static final NeutralModeValue MODE = NeutralModeValue.Brake;
        public static final int CURRENT_LIMIT = 60; //
        public static final boolean IS_INVERTED = false;

        public static final int UPDATE_FREQ = 50;

        // public static final double MAX_VELOCITY_MPS = 2.0; //
        // public static final double MAX_ACCELERATION_MPS2 = 8.0; // not used currently

        public static final double MM_CRUISE_VELCOCITY_UP = 100; //
        public static final double MM_ACCELERATION_UP = 100; //

        public static final double MM_CRUISE_VELCOCITY_DOWN = 1; //
        public static final double MM_ACCELERATION_DOWN = 1; //

        public static final double TICKS_PER_REV = 4000; //
        public static final double GEAR_RATIO = 3; //
        public static final double PULLEY_DIAMETER = 2.005; // should be chain pitch * number of teeth / pi
        public static final double kRotationToInches = PULLEY_DIAMETER * Math.PI / GEAR_RATIO;

        // the following are in inches
        public static final double STOWED_HEIGHT = 0;
        public static final double STATION_HEIGHT =
                15; // Home Field is 1in higher than official field - official is 15.4
        public static final double LEVEL_TWO_HEIGHT =
                8.625; // 10.9; // was 12, 7 This is slightly away from the reef for clearance //
        public static final double LEVEL_THREE_HEIGHT = 1.5; // 15 //TODO l3 height
        public static final double LEVEL_FOUR_HEIGHT = 24.6; // 29.625; //

        public static final double ALGAE_LOW_HEIGHT = 6.7;
        public static final double ALGAE_HIGH_HEIGHT = 16;

        public static final double BARGE_HEIGHT = 33;

        public static final double MAX_ELEVATOR_HEIGHT = 34;

        public static final double HOMING_SPEED = -0.05;

        public static final double LEVEL_TWO_ROT = LEVEL_TWO_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double STATION_ROT = STATION_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double LEVEL_THREE_ROT = LEVEL_THREE_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double LEVEL_FOUR_ROT = LEVEL_FOUR_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double ALGAE_LOW_ROT = ALGAE_LOW_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double ALGAE_HIGH_ROT = ALGAE_HIGH_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double MAX_ELEVATOR_ROT = MAX_ELEVATOR_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double BARGE_ROT = BARGE_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double STOWED_ROT = STOWED_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);

        // public static final double STATION_ROT = 3.868;
        // public static final double LEVEL_TWO_ROT = 3.868;
        // public static final double LEVEL_THREE_ROT = 10;
        // public static final double LEVEL_FOUR_ROT = 15.0;
        // public static final double MAX_ELEVATOR_ROT = 15.5; // 15.65
        // public static final double STOWED_ROT = 0;

        public static final double ELEVATOR_OFFSET = 0.075;

        // TODO: change all of these values to match true elevator gains

        public static final double kG = 0.29; // 0.3
        public static final double kS = 0.11; // 0
        public static final double kV = 0.1; // 0
        public static final double kA = 0.0; // 0
        public static final double kP = 0.4; // 0.4
        public static final double kI = 0.0; // 0
        public static final double kD = 0.0; // 0
    }

    public static final class ArmConstants {
        public static final int ARM_KRAKEN_ID = 22;
        public static final int CURRENT_LIMIT = 20;

        public static final double ARM_GEAR_RATIO = 60; // ?

        public static final double ARM_LEVEL_4_ROT =
                Units.degreesToRotations(156) * ARM_GEAR_RATIO - 0.52; // -170 //-180 //0155.94 //-161
        public static final double ARM_LEVEL_3_ROT =
                Units.degreesToRotations(147) * ARM_GEAR_RATIO - 0.3; // was -78 //79.08 // -66

        public static final double ARM_LEVEL_2_ROT =
                Units.degreesToRotations(-66) * ARM_GEAR_RATIO; // was -85, then -74.455078125[\] //79.08
        public static final double ARM_INTAKE_ROT =
                Units.degreesToRotations(-15) * ARM_GEAR_RATIO; //  was 61 //-299 // -311
        public static final double ARM_STOWED_ROT = Units.degreesToRotations(0) * ARM_GEAR_RATIO; // should be 0

        public static final double ARM_ALGAE_LOW = Units.degreesToRotations(73) * ARM_GEAR_RATIO;
        public static final double ARM_ALGAE_HIGH = Units.degreesToRotations(82) * ARM_GEAR_RATIO;
        public static final double ARM_BARGE = Units.degreesToRotations(160) * ARM_GEAR_RATIO - 2.1;
        // public static final double ARM_CLIMB = Units.degreesToRotations(-50) * ARM_GEAR_RATIO;
        public static final double ARM_LOLLIPOP = Units.degreesToRotations(-50) * ARM_GEAR_RATIO;

        public static final double ARM_ADJUST_INCREMENT = 0.05;

        public static final double UPDATE_FREQ = 50;

        // TODO: tune pid

        public static final double kG = 0.0; // 0.35;
        public static final double kS = 0.0; // 0.15;
        public static final double kV = 0.0; // 0.2
        public static final double kA = 0.0; // 0.07
        public static final double kP = 0.0; // 0.3
        public static final double kI = 0.0;
        public static final double kD = 0.0; // 0.05

        public static final double MM_MAX_CRUISE_VELOCITY = 2;
        public static final double MM_MAX_CRUISE_ACCELERATION = 0.5;
    }

    public static final class ClimbConstants {
        public static final int CLIMB_MOTOR_ID = 24;
        public static final int CLIMB_MOTOR_CURRENT_LIMIT = 40;
        public static final double CLIMB_MOTOR_VOLTAGE_COMP = 10;
        public static final double CLIMB_VALUE = 0.7;

        public static final double CLIMB_ROTS = -126.0;
    }

    public static final class IntakeConstants {
        // CANID for pivot
        public static final int PIVOT_ID = 40; // real CANID

        // public static final int PIVOT_GEAR_RATIO = 60;
        public static final NeutralModeValue MODE = NeutralModeValue.Brake;
        public static final int ROLLER_CURRENT_LIMIT = 30;
        public static final int PIVOT_CURRENT_LIMIT = 30;

        // PID for pivot
        public static final double PIVOT_kP = 0.5;
        public static final double PIVOT_kI = 0.0;
        public static final double PIVOT_kD = 0.0;

        public static final double PIVOT_MIN_OUTPUT = -0.5;
        public static final double PIVOT_MAX_OUTPUT = 0.5;

        public static final double PIVOT_GEARING = 60; // 25:1 reduction gear ratio

        public static final double PIVOT_OUTTAKE_ROT =
                Units.degreesToRotations(10.0) * PIVOT_GEARING; // TODO: find outtake rot in motor rotations
        public static final double PIVOT_INTAKE_ROT =
                Units.degreesToRotations(105.0) * PIVOT_GEARING; // TODO: find intake rot in motor rotations
        public static final double PIVOT_STOWED_ROT =
                Units.degreesToRotations(0.0) * PIVOT_GEARING; // TODO: find stowed rot in motor rotations
        public static final double PIVOT_ALGAE_ROT =
                Units.degreesToRotations(40.0) * PIVOT_GEARING; // TODO: find stowed rot in motor rotations

        // CAN ID for roller
        public static final int ROLLER_ID = 41;
    }

    public static final class EndEffectorConstants {
        public static final int END_EFFECTOR_ID = 23;

        public static final double PULL_SPEED = -0.3;

        public static final double PUSH_SPEED = 0.4;
        public static final double ALGAE_PULL_SPEED = 0.8;
        public static final double ALGAE_PUSH_SPEED = -1.0;

        public static final double HOLD_SPEED = -0.075;

        public static final int END_SENSOR_CHANNEL = 0;
    }
}
