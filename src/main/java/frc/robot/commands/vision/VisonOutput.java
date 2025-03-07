package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class VisonOutput extends Command {

    public VisonOutput(){

    }

    @Override
    public void execute(){
        double tx = LimelightHelpers.getTX("");// Horizontal offset from crosshair to target in degrees
		double ty = LimelightHelpers.getTY("");// Vertical offset from crosshair to target in degrees
        double ta = LimelightHelpers.getTA("");// Target area (0% of image to 100% of image)
		double id = LimelightHelpers.getFiducialID("");
        double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
        double tync = LimelightHelpers.getTYNC("");  // Vertical offset from principal pixel/point to target in degrees
        double[] tag = LimelightHelpers.getT2DArray("");
        Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace("");
        Pose2d pose2d =LimelightHelpers.toPose2D(LimelightHelpers.getBotPose(""));
        Rotation3d rot = pose.getRotation();
    }
   
}
