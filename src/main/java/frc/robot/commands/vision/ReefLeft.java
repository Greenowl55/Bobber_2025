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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;

public class ReefLeft extends Command {

	private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
	.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

private CommandSwerveDrivetrain m_drive;

private double rotSetpoint = -1;
private double xsetpoint = -0.17; //TODO
private double zsetpoint = -.45; //TODO
private boolean tagVisible = false;

// PID Controllers for alignment
private final PIDController rotationPID;
private final PIDController strafePID;
private final PIDController distancePID;

private double rotationP = 0.025;
private double rotationI = 0;
private double rotationD = 0;

private double strafeP = 1.5;
private double strafeI = 0.03;
private double strafeD = 0.002;

private double distanceP = 1.5;
private double distanceI = 0.03;
private double distanceD = 0.002;


private double botX = 0;
private double botZ = 0;
private double botYaw = 0;

private double distanceError = 0;
private double strafeError = 0;
private double rotationError = 0;

private double distanceOutput = 0;
private double strafeOutput = 0;
private double rotationOutput = 0;
//public static Pose2d tPose2d(double[] inData);

public ReefLeft(CommandSwerveDrivetrain drive_subsystem) {
addRequirements(drive_subsystem);
m_drive = drive_subsystem;

// Configure PID controllers with gains
rotationPID = new PIDController(rotationP, rotationI, rotationD );
strafePID = new PIDController(strafeP, strafeI, strafeD);
distancePID = new PIDController(distanceP, distanceI, distanceD);
SmartDashboard.putData(this);

// Set tolerances for when we consider ourselv+es "aligned"
rotationPID.setTolerance(2);
strafePID.setTolerance(.02);
distancePID.setTolerance(.05);
}

@Override
public void initialize() {
rotationPID.reset();
strafePID.reset();
distancePID.reset();

	rotationPID.setSetpoint(rotSetpoint);
	strafePID.setSetpoint(this.xsetpoint);
	distancePID.setSetpoint(this.zsetpoint);
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
double[] fieldpose = LimelightHelpers.getBotPose_TargetSpace(Constants.LIMELIGHT4_NAME);
Pose3d botPose = LimelightHelpers.getBotPose3d_TargetSpace(Constants.LIMELIGHT4_NAME);

boolean tagFound = false;
tagVisible = false;
for (int tag : Constants.REEF_TAGS) {
	if (id == tag) {
		tagFound = true;
		break;
	}
}
if (tagFound == true) { // Calculate control outputs
	this.tagVisible = true;
	botX = botPose.getX(); //offset from tag in meters
	botZ = botPose.getZ(); //distance from tag in meters
	double yawtotag = Math.toDegrees(botPose.getRotation().getY()); // angle to tag in degrees
	this.botYaw = yawtotag;
	
	double xoffset = xsetpoint - botX;
	double zoffset = zsetpoint - botZ;
	double rotoffset = rotSetpoint - yawtotag;
	
	this.strafeError = xoffset;
	this.distanceError = zoffset;
	this.rotationError = rotoffset;

	double rotationOutput = -rotationPID.calculate(yawtotag) * Math.PI /2;
	double strafeOutput = -strafePID.calculate(botX);
	double forwardOutput = distancePID.calculate(botZ);
	

	//limit speed
	rotationOutput = Math.max(-Math.PI, Math.min(Math.PI, rotationOutput));
	strafeOutput = Math.max(-1, Math.min(1, strafeOutput));
	forwardOutput = Math.max(-1, Math.min(1, forwardOutput));

	double outputMin = 0.15;
	if (rotationOutput > 0 && rotationOutput < outputMin) {
		rotationOutput = outputMin;
	} else if (rotationOutput < 0 && rotationOutput > -outputMin) {
		rotationOutput = -outputMin;
	}
	if (strafeOutput > 0 && strafeOutput < outputMin) {
		strafeOutput = outputMin;
	} else if (strafeOutput < 0 && strafeOutput > -outputMin) {
		strafeOutput = -outputMin;
	}
	if (forwardOutput > 0 && forwardOutput < outputMin) {
		forwardOutput = outputMin;
	} else if (forwardOutput < 0 && forwardOutput > -outputMin) {
		forwardOutput = -outputMin;
	}
	
	if (rotationPID.atSetpoint()) {
		rotationOutput = 0;
	}

	if (strafePID.atSetpoint()) {
		strafeOutput = 0;
	}

	if (distancePID.atSetpoint()) {
		forwardOutput = 0;
	}

	this.strafeOutput = strafeOutput;
	this.rotationOutput = rotationOutput;
	this.distanceOutput = forwardOutput;

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