package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class TargetDebug extends SubsystemBase {

    private String limelightName;

    public TargetDebug(String limelightName) {
        SmartDashboard.putData(this);
        this.limelightName = limelightName;

    }



    public Pose3d getBotPoseTargetSpace() {
        if (LimelightHelpers.getTV(limelightName)) {
            return LimelightHelpers.getBotPose3d_TargetSpace(limelightName);
        } else {
            return null;
        }
    }

    public double getOffsetToTarget() {
        Pose3d pose = getBotPoseTargetSpace();
        if (pose != null) {
            return pose.getX();
        } else {
            return Double.NEGATIVE_INFINITY;
        }
    }

    public double getDistanceToTarget() {
        Pose3d pose = getBotPoseTargetSpace();
        if (pose != null) {
            return pose.getZ();
        } else {
            return Double.NEGATIVE_INFINITY;
        }
    }

    public double getRotationToTarget() {
        Pose3d pose = getBotPoseTargetSpace();
        if (pose != null) {
            return Math.toDegrees(pose.getRotation().getY());
        } else {
            return Double.NEGATIVE_INFINITY;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("offsetToTarget", this::getOffsetToTarget, null);
        builder.addDoubleProperty("distanceToTarget", this::getDistanceToTarget, null);
        builder.addDoubleProperty("rotationToTarget", this::getRotationToTarget, null);
    }

}
