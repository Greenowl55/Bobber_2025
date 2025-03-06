// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;

public class ReefLeft extends Command {

        private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private CommandSwerveDrivetrain m_drive;

	private final double rotSetpoint = 0; //TODO
	private final double txSetpoint = 0; //TODO
	private final double taSetpoint = 4; //TODO

	// PID Controllers for alignment
	private final PIDController rotationPID;
	private final PIDController strafePID;
	private final PIDController distancePID;
	//public static Pose2d tPose2d(double[] inData);

	public ReefLeft(CommandSwerveDrivetrain drive_subsystem) {
		addRequirements(drive_subsystem);
		m_drive = drive_subsystem;

		// Configure PID controllers with gains
		rotationPID = new PIDController(0.025, 0, 0 );
		strafePID = new PIDController(0.1, 0.01, 0.002);
		distancePID = new PIDController(0.1, 0.01, 0.002);
		addRequirements(drive_subsystem);

		// Set tolerances for when we consider ourselv+es "aligned"
		rotationPID.setTolerance(1);
		strafePID.setTolerance(1);
		distancePID.setTolerance(1);
	}

	@Override
	public void initialize() {
		rotationPID.reset();
		strafePID.reset();
		distancePID.reset();
		
			rotationPID.setSetpoint(rotSetpoint);
			strafePID.setSetpoint(txSetpoint);
			distancePID.setSetpoint(taSetpoint);
	}

	@Override
	public void execute() {
		double tx = LimelightHelpers.getTX("");// Horizontal offset from crosshair to target in degrees
		double ty = LimelightHelpers.getTY("");// Vertical offset from crosshair to target in degrees
        double ta = LimelightHelpers.getTA("");// Target area (0% of image to 100% of image)
		double id = LimelightHelpers.getFiducialID("");
        double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
        double tync = LimelightHelpers.getTYNC("");  // Vertical offset from principal pixel/point to target in degrees
		Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace("");
		double rot = Math.toDegrees(pose.getRotation().getY());

		boolean tagFound = false;
		for (int tag : Constants.REEF_TAGS) {
			if (id == tag) {
				tagFound = true;
				break;
			}
		}
		if (tagFound == true) { // Calculate control outputs
			
			double rotationOutput = rotationPID.calculate(rot);
			double strafeOutput = strafePID.calculate(tx);
			double forwardOutput = distancePID.calculate(ta);
			
			System.out.println("tx: " + tx  + ", ta "+ ta + ", rot " + rot);
			System.out.println("Error: " + (txSetpoint - tx) + ", " + (taSetpoint - ta) + ", " + (rotSetpoint - rot));
			System.out.println("output: " + strafeOutput + ", " + forwardOutput + ", " + rotationOutput);
			System.out.println("setpoint: " + strafePID.atSetpoint() + ", " + distancePID.atSetpoint() + ", " + rotationPID.atSetpoint());
			// Apply combined movement
			m_drive.setControl(
					drive
							.withVelocityX(forwardOutput) // Forward/backward
							.withVelocityY(strafeOutput) // Left/right
							.withRotationalRate(rotationOutput) ); // Rotation
		} else {
			// If we don't see the correct tag, stop moving
			m_drive.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
		}

		
	}

	// @Override
	// public void end(boolean interrupted) {
	// 	m_drive.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
	// }

	@Override
	public boolean isFinished() {
		// Command finishes when we're aligned with the target
		return rotationPID.atSetpoint() && strafePID.atSetpoint() && distancePID.atSetpoint();
	}

	@Override
	public boolean runsWhenDisabled() {
		return false;
	}
    
    
}
