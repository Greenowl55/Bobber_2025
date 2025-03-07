// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.vision;

import java.util.EnumSet;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ReefRight extends Command {

	private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
	.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private CommandSwerveDrivetrain m_drive;

	private final double rotSetpoint = 0; //TODO 
	private final double txSetpoint = -12.5; // TODO
	private final double taSetpoint = 3.5; //TODO

	// PID Controllers for alignment
	private final PIDController rotationPID;
	private final PIDController strafePID;
	private final PIDController distancePID;
	//public static Pose2d tPose2d(double[] inData);

	public DoubleArrayPublisher rotationPIDPublisher;
	public DoubleArrayPublisher straifPIDPublisher;
	public DoubleArrayPublisher forwardPIDPublisher;

	public ReefRight(CommandSwerveDrivetrain drive_subsystem) {
		addRequirements(drive_subsystem);
		m_drive = drive_subsystem;

		// Configure PID controllers with gains
		rotationPID = new PIDController(0.1, 0.05, 0 );

		NetworkTableEntry rotationPIDEntry = NetworkTableInstance.getDefault().getTable("PID").getEntry("rotationPID");
		rotationPIDEntry.setDoubleArray(new double[]{rotationPID.getP(), rotationPID.getI(), rotationPID.getD()});
		NetworkTableInstance.getDefault().getTable("PID")
		.addListener("rotationPID", EnumSet.of(Kind.kValueRemote), (nt, key, event) -> {
			double[] data = event.valueData.value.getDoubleArray();
			rotationPID.setP(data[0]);
			rotationPID.setI(data[1]);
			rotationPID.setD(data[2]);
		});

		strafePID = new PIDController(0.1, 0.01, 0.002);

		NetworkTableEntry strafePIDEntry = NetworkTableInstance.getDefault().getTable("PID").getEntry("strafePID");
		strafePIDEntry.setDoubleArray(new double[]{strafePID.getP(), strafePID.getI(), strafePID.getD()});
		NetworkTableInstance.getDefault().getTable("PID")
		.addListener("strafePID", EnumSet.of(Kind.kValueRemote), (nt, key, event) -> {
			double[] data = event.valueData.value.getDoubleArray();
			strafePID.setP(data[0]);
			strafePID.setI(data[1]);
			strafePID.setD(data[2]);
		});

		distancePID = new PIDController(0.1, 0.01, 0.002);

		NetworkTableEntry forwardPIDEntry = NetworkTableInstance.getDefault().getTable("PID").getEntry("fowardPID");
		forwardPIDEntry.setDoubleArray(new double[]{distancePID.getP(), distancePID.getI(), distancePID.getD()});
		NetworkTableInstance.getDefault().getTable("PID")
		.addListener("forwardPID", EnumSet.of(Kind.kValueRemote), (nt, key, event) -> {
			double[] data = event.valueData.value.getDoubleArray();
			distancePID.setP(data[0]);
			distancePID.setI(data[1]);
			distancePID.setD(data[2]);
		});

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
		double tx = LimelightHelpers.getTX(Constants.LIMELIGHT4_NAME);// Horizontal offset from crosshair to target in degrees
		double ty = LimelightHelpers.getTY(Constants.LIMELIGHT4_NAME);// Vertical offset from crosshair to target in degrees
        double ta = LimelightHelpers.getTA(Constants.LIMELIGHT4_NAME);// Target area (0% of image to 100% of image)
		double id = LimelightHelpers.getFiducialID(Constants.LIMELIGHT4_NAME);
        double txnc = LimelightHelpers.getTXNC(Constants.LIMELIGHT4_NAME);  // Horizontal offset from principal pixel/point to target in degrees
        double tync = LimelightHelpers.getTYNC(Constants.LIMELIGHT4_NAME);  // Vertical offset from principal pixel/point to target in degrees
		Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT4_NAME);
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
			
			// System.out.println("tx: " + tx  + ", ta "+ ta + ", rot " + rot);
			// System.out.println("Error: " + (txSetpoint - tx) + ", " + (taSetpoint - ta) + ", " + (rotSetpoint - rot));
			// System.out.println("output: " + strafeOutput + ", " + forwardOutput + ", " + rotationOutput);
			// System.out.println("setpoint: " + strafePID.atSetpoint() + ", " + distancePID.atSetpoint() + ", " + rotationPID.atSetpoint());
			// Apply combined movement
			m_drive.setControl(
					drive
							//.withVelocityX(forwardOutput) // Forward/backward
							//.withVelocityY(strafeOutput) // Left/right
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
