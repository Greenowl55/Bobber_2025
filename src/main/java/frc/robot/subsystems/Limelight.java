// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
	private final NetworkTable m_limelightTable;
	private final CommandSwerveDrivetrain m_swerve;

	// Constants for Limelight mounting
	private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 25.0; // Adjust based on your mounting
	private static final double LIMELIGHT_HEIGHT_METERS = 0.5; // Adjust based on your robot
	private static final double TARGET_HEIGHT_METERS =
			2.1; // Adjust based on your target (e.g., AprilTag height)

	// Add camera offset constants from robot center
	private static final double CAMERA_OFFSET_X_METERS = 0.0; // Positive X is forward
	private static final double CAMERA_OFFSET_Y_METERS = 0.0; // Positive Y is left
	private static final double CAMERA_OFFSET_Z_METERS = 0.0; // Height from ground to camera lens

	// Create a Transform2d to represent the camera offset
	private final Transform2d cameraToRobot =
			new Transform2d(
					new Translation2d(CAMERA_OFFSET_X_METERS, CAMERA_OFFSET_Y_METERS),
					new Rotation2d(0.0) // Add rotation offset if camera is rotated relative to robot
					);

	public Limelight(CommandSwerveDrivetrain swerve) {
		m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
		m_swerve = swerve;
		configureForAprilTags();

		// Add camera offsets to SmartDashboard for tuning
		SmartDashboard.putNumber("Limelight/Camera Offset X", CAMERA_OFFSET_X_METERS);
		SmartDashboard.putNumber("Limelight/Camera Offset Y", CAMERA_OFFSET_Y_METERS);
		SmartDashboard.putNumber("Limelight/Camera Offset Z", CAMERA_OFFSET_Z_METERS);
	}

	@Override
	public void periodic() {

		if (hasTarget()) {
			updatePoseWithVision();

			// Update SmartDashboard with vision data
			SmartDashboard.putNumber("Limelight Distance", getTargetDistance());
			SmartDashboard.putNumber("Limelight TX", m_limelightTable.getEntry("tx").getDouble(0.0));
			SmartDashboard.putNumber("Limelight TY", m_limelightTable.getEntry("ty").getDouble(0.0));
			SmartDashboard.putBoolean("Has Target", true);

			// Add camera offset debug info
			var robotPose = m_swerve.getPose();
			var cameraPose = robotPose.transformBy(cameraToRobot);
			SmartDashboard.putNumber("Limelight/Camera Global X", cameraPose.getX());
			SmartDashboard.putNumber("Limelight/Camera Global Y", cameraPose.getY());
		} else {
			SmartDashboard.putBoolean("Has Target", false);
		}
	}

	public boolean hasTarget() {
		return m_limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
	}

	public double getTargetDistance() {
		if (!hasTarget()) {
			return 0.0;
		}

		// Get the vertical offset angle
		double targetOffsetAngle_Vertical = m_limelightTable.getEntry("ty").getDouble(0.0);

		// Calculate distance using trigonometry
		double angleToGoalRadians =
				Units.degreesToRadians(LIMELIGHT_MOUNT_ANGLE_DEGREES + targetOffsetAngle_Vertical);
		double distance =
				(TARGET_HEIGHT_METERS - LIMELIGHT_HEIGHT_METERS) / Math.tan(angleToGoalRadians);

		return distance;
	}

	private void updatePoseWithVision() {
		// Get target info from Limelight
		double[] botpose = m_limelightTable.getEntry("botpose").getDoubleArray(new double[6]);

		if (botpose.length >= 6) {
			// Extract position and rotation from botpose
			Pose2d visionPose =
					new Pose2d(new Translation2d(botpose[0], botpose[1]), Rotation2d.fromDegrees(botpose[5]));

			// Transform the camera pose to robot pose using the offset
			Pose2d robotPose = visionPose.transformBy(cameraToRobot.inverse());

			// Get the latency from the Limelight
			double latencySeconds =
					(m_limelightTable.getEntry("tl").getDouble(0.0)
									+ m_limelightTable.getEntry("cl").getDouble(0.0))
							/ 1000.0;

			// Update the swerve odometry with vision measurement
			m_swerve.addVisionMeasurement(robotPose, latencySeconds);
		}
	}

	// Method to configure Limelight for AprilTag detection
	public void configureForAprilTags() {
		m_limelightTable
				.getEntry("pipeline")
				.setNumber(0); // Assuming pipeline 0 is configured for AprilTags
		m_limelightTable.getEntry("camMode").setNumber(0); // Set to vision processing mode
		m_limelightTable.getEntry("ledMode").setNumber(0); // LED mode to pipeline default
	}

	public Pose2d getTargetPose() {
		double[] botpose = m_limelightTable.getEntry("botpose").getDoubleArray(new double[6]);

		if (botpose.length >= 6) {
			return new Pose2d(
					new Translation2d(botpose[0], botpose[1]), Rotation2d.fromDegrees(botpose[5]));
		}
		return new Pose2d();
	}
}
