package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.util.vision.LimelightHelpers;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AutoAlign {

    private Supplier<Pose2d> poseSupplier;

    private PIDController reefStrafeSpeedController =
            new PIDController(AlignConstants.REEF_kP, AlignConstants.REEF_kI, AlignConstants.REEF_kD);
    private PIDController reefRotationSpeedController =
            new PIDController(AlignConstants.ROT_REEF_kP, AlignConstants.ROT_REEF_kI, AlignConstants.ROT_REEF_kD);

    private double alignSpeedStrafe = 0;
    private int currentReefAlignTagID = 18; // -1
    private Map<Integer, Pose3d> tagPoses3d = getTagPoses();

    public AutoAlign(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    public Map<Integer, Pose3d> getTagPoses() {
        Map<Integer, Pose3d> tagPoses = new HashMap<Integer, Pose3d>();
        for (int tag : FieldConstants.RED_REEF_TAG_IDS) {
            tagPoses.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        for (int tag : FieldConstants.BLUE_REEF_TAG_IDS) {
            tagPoses.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        return tagPoses;
    }

    private Rotation2d getTagAngle(int tagID) {

        if (tagPoses3d.containsKey(tagID)) {
            Pose3d tagPose = tagPoses3d.get(tagID);
            return new Rotation2d(tagPose.getRotation().getZ());
        } else return new Rotation2d(0);
    }

    public Rotation2d getAlignAngleReef() {
        currentReefAlignTagID = getClosestAprilTag(
                RobotContainer.isRedAlliance() ? FieldConstants.RED_REEF_TAG_IDS : FieldConstants.BLUE_REEF_TAG_IDS,
                poseSupplier.get());

        return getTagAngle(currentReefAlignTagID);
    }

    private int getClosestAprilTag(int[] tagIDs, Pose2d robotPose) {
        double minDistance = Double.POSITIVE_INFINITY;
        int closestTagID = -1;

        // iterates through all tag IDs
        for (int i : tagIDs) {
            if (tagPoses3d.containsKey(i)) {
                Pose3d tagPose = tagPoses3d.get(i);

                // distance between robot pose and april tag
                double distance = tagPose.getTranslation()
                        .toTranslation2d()
                        .minus(robotPose.getTranslation())
                        .getNorm();

                if (distance < minDistance) {
                    closestTagID = i;
                    minDistance = distance;
                }
            }
        }

        return closestTagID;
    }

    public double getAlignStrafeSpeedPercent(double setPoint) {
        double tx = LimelightHelpers.getTX(VisionConstants.LL_NAME);
        double txError = tx - setPoint;

        // if the drivetrain isn't yet rotationally aligned, this affects the tx
        boolean withinRotRolerance = Math.abs(getAlignAngleReef()
                        .minus(poseSupplier.get().getRotation())
                        .getDegrees())
                < AlignConstants.ALIGN_ROT_TOLERANCE_DEGREES;
        Logger.recordOutput("Align/IsWithinRotTolerance", withinRotRolerance);

        boolean isValid = llIsValid() && withinRotRolerance;
        if (isValid) {
            // multiply error by kP to get the speed
            alignSpeedStrafe = -reefStrafeSpeedController.calculate(tx, setPoint);
            alignSpeedStrafe += AlignConstants.ALIGN_KS * Math.signum(alignSpeedStrafe);
        } else {
            // reduce the current align speed by 1/4 each tick
            // this prevents it from suddenly stopping and starting when it loses sight of the tag
            alignSpeedStrafe *= AlignConstants.ALIGN_DAMPING_FACTOR;
        }

        Logger.recordOutput("Align/Strafe Speed", alignSpeedStrafe);
        Logger.recordOutput("Align/tx", tx);
        Logger.recordOutput("Align/tx Error", txError);

        return alignSpeedStrafe;
    }

    // public double getAlignRotationSpeedPercent(double angle) {
    //     Rotation2d robotAngle = poseSupplier.get().getRotation();
    //     Rotation2d tagAngle = getTagAngle(currentReefAlignTagID);
    //     Rotation2d rotationError = robotAngle.minus(tagAngle);

    //     boolean isValid = llIsValid();
    //     if(isValid) {

    //     }
    //     else {

    //     }
    //     return 0;
    // }

    private boolean llIsValid() {
        boolean valid = LimelightHelpers.getTargetCount(VisionConstants.LL_NAME) == 1
                && LimelightHelpers.getFiducialID(VisionConstants.LL_NAME) == currentReefAlignTagID;
        Logger.recordOutput("Align/Valid", valid);
        return valid;
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds(
            double tx, double ySpeed, Rotation2d gyroAngle, double maxSpeed) {
        Logger.recordOutput("Align/Timestamp", System.currentTimeMillis());
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                getAlignStrafeSpeedPercent(tx) * maxSpeed,
                ySpeed,
                0,
                gyroAngle); // getAlignStrafeSpeedPercent(tx) * maxSpeed
    }
}
