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

    public Double getOffsetToTarget() {
        Pose3d pose = getBotPoseTargetSpace();
        if (pose != null) {
            return pose.getX();
        } else {
            return null;
        }
    }

    public Double getDistanceToTarget() {
        Pose3d pose = getBotPoseTargetSpace();
        if (pose != null) {
            return pose.getZ();
        } else {
            return null;
        }
    }

    public Double getRotationToTarget() {
        Pose3d pose = getBotPoseTargetSpace();
        if (pose != null) {
            return pose.getRotation().getAngle();
        } else {
            return null;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("offsetToTarget", this::getOffsetToTarget, null);
        builder.addDoubleProperty("distanceToTarget", this::getDistanceToTarget, null);
        builder.addDoubleProperty("rotationToTarget", this::getRotationToTarget, null);
    }

}
