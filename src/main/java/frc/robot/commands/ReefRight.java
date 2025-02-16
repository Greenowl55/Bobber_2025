// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

public class ReefRight extends Command {
	private final CommandSwerveDrivetrain m_drivetrain;
	private final Limelight m_limelight;
	private final PIDController m_xController;
	private final PIDController m_yController;
	private final PIDController m_rotationController;

	// Define desired offsets from AprilTag
	private static final double TARGET_OFFSET_X =
			-1.0; // Meters in front of tag (negative to stop before tag)
	private static final double TARGET_OFFSET_Y = 0.5; // Meters to right of tag

	// Define alignment tolerances
	private static final double POSITION_TOLERANCE = 0.02; // 2cm
	private static final double ROTATION_TOLERANCE = Math.toRadians(2.0); // 2 degrees

	private final SwerveRequest.FieldCentric m_request =
			new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	public ReefRight(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
		m_drivetrain = drivetrain;
		m_limelight = limelight;

		m_xController = new PIDController(1.0, 0, 0);
		m_yController = new PIDController(1.0, 0, 0);
		m_rotationController = new PIDController(0.1, 0, 0);
		m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

		// Set tolerance for position and rotation
		m_xController.setTolerance(POSITION_TOLERANCE);
		m_yController.setTolerance(POSITION_TOLERANCE);
		m_rotationController.setTolerance(ROTATION_TOLERANCE);

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		m_xController.reset();
		m_yController.reset();
		m_rotationController.reset();
	}

	@Override
	public void execute() {
		if (!m_limelight.hasTarget()) {
			m_drivetrain.applyRequest(
					() -> m_request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
			return;
		}

		// Get target pose
		var targetPose = m_limelight.getTargetPose();

		// Calculate desired robot pose with offsets
		var desiredPose =
				new Pose2d(
						targetPose.getX() + TARGET_OFFSET_X,
						targetPose.getY() + TARGET_OFFSET_Y,
						targetPose.getRotation());

		// Get current robot pose
		var robotPose = m_drivetrain.getPose();

		// Calculate errors
		double xError = desiredPose.getX() - robotPose.getX();
		double yError = desiredPose.getY() - robotPose.getY();
		double rotationError =
				desiredPose.getRotation().getRadians() - robotPose.getRotation().getRadians();

		// Calculate control outputs
		double xSpeed = m_xController.calculate(robotPose.getX(), desiredPose.getX());
		double ySpeed = m_yController.calculate(robotPose.getY(), desiredPose.getY());
		double rotationSpeed =
				m_rotationController.calculate(
						robotPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());

		// Log data to SmartDashboard
		SmartDashboard.putNumber("Vision/Target X Error", xError);
		SmartDashboard.putNumber("Vision/Target Y Error", yError);
		SmartDashboard.putNumber("Vision/Target Rotation Error", rotationError);
		SmartDashboard.putNumber("Vision/Desired X", desiredPose.getX());
		SmartDashboard.putNumber("Vision/Desired Y", desiredPose.getY());

		// Apply to drivetrain
		m_drivetrain.applyRequest(
				() ->
						m_request
								.withVelocityX(xSpeed)
								.withVelocityY(ySpeed)
								.withRotationalRate(rotationSpeed));
	}

	@Override
	public void end(boolean interrupted) {
		m_drivetrain.applyRequest(
				() -> m_request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
	}

	@Override
	public boolean isFinished() {
		if (!m_limelight.hasTarget()) return false;

		return m_xController.atSetpoint()
				&& m_yController.atSetpoint()
				&& m_rotationController.atSetpoint();
	}
}
