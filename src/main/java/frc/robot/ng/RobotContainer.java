// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.ng;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Telemetry;
import frc.robot.Autos.Drive1;
import frc.robot.ng.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.ng.subsystems.*;

public class RobotContainer {

	Elevator m_elevator = new Elevator();
	FishHook m_fish_hook = new FishHook();

	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
	private double MaxAngularRate = RotationsPerSecond.of(0.75)
			.in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.05)
			.withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
			.withDriveRequestType(
					DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final Telemetry logger = new Telemetry(MaxSpeed);

	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandGenericHID coDriverController = new CommandGenericHID(1);
	private final Joystick codriverJoystick = new Joystick(1);

	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

	/* Path follower */
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		autoChooser = AutoBuilder.buildAutoChooser("Tests");
		SmartDashboard.putData("Auto Mode", autoChooser);

		autoChooser.setDefaultOption("Do Nothing", new PrintCommand("Do Nothing"));
		autoChooser.addOption("Drive", new Drive1(drivetrain));
		autoChooser.addOption("forward", drivetrain.getAutoPath("Drive"));

		configureBindings();

		// Configure the pathplanner named commands
		// CommandRegristry.setupNamedCommands(m_elevator, m_fish_hook);
	}

	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(
						() -> drive
								.withVelocityX(
										-driverController.getLeftY()
												* MaxSpeed) // Drive forward with negative Y (forward)
								.withVelocityY(
										-driverController.getLeftX()
												* MaxSpeed) // Drive left with negative X (left)
								.withRotationalRate(
										-driverController.getRightX()
												* MaxAngularRate) // Drive counterclockwise with negative X (left)
				));

		// TODO Find button for:
		// driverController.a().toggleOnTrue(drivetrain.applyRequest(() ->
		// brake));
		// joystick.b().whileTrue(drivetrain.applyRequest(() ->
		// point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
		// -joystick.getLeftX()))
		// ));

		driverController
				.pov(0)
				.whileTrue(
						drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
		driverController
				.pov(180)
				.whileTrue(
						drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		driverController
				.back()
				.and(driverController.y())
				.whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		driverController
				.back()
				.and(driverController.x())
				.whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		driverController
				.start()
				.and(driverController.y())
				.whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		driverController
				.start()
				.and(driverController.x())
				.whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// reset the field-centric heading on left bumper press
		driverController.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		drivetrain.registerTelemetry(logger::telemeterize);

		// Driver Buttons

		// Coral Scoring
		// driverController
		// .a()
		// .onTrue(new CMD_ScoringState(ScoringState.State.L1, m_elevator,
		// m_fish_hook));
		// driverController
		// .b()
		// .onTrue(new CMD_ScoringState(ScoringState.State.L2, m_elevator,
		// m_fish_hook));
		// driverController
		// .x()
		// .onTrue(new CMD_ScoringState(ScoringState.State.L3, m_elevator,
		// m_fish_hook));
		// driverController
		// .y()
		// .onTrue(new CMD_ScoringState(ScoringState.State.L4, m_elevator,
		// m_fish_hook));

		// TODO right bumper for right side of reef alignment

		// TODO left bumper for left side of reef alignment

		// Idle to bottom
		// driverController
		// .button(7)
		// .onTrue(new CMD_ScoringState(ScoringState.State.BOTTOM, m_elevator,
		// m_fish_hook));

		// driverController.rightBumper().whileTrue(new ElevatorUp(m_elevator));

		// driverController.leftBumper().whileTrue(new ElevatorDown(m_elevator));
		// Co-Driver Buttons

		coDriverController
				.button(1) // trigger
				.whileTrue(new CoralManual(m_fish_hook));

		coDriverController
				.button(2) // Bottom Face Button
				.onTrue(new CoralAuto(m_fish_hook));

		coDriverController
				.button(3) // left face buttom
				.whileTrue(new AlgaeIn(m_fish_hook));

		coDriverController
				.button(4) // right face button
				.whileTrue(new AlgaeOut(m_fish_hook));

		// m_fish_hook.run(new tilt(() -> coDriverController.getY().m_Fishhook));

		m_fish_hook.setDefaultCommand(

				new FishHookAngle(
						m_fish_hook, () -> codriverJoystick.getRawAxis(Joystick.AxisType.kY.value) * 0.1));

		/*
		 * TODO
		 * climber
		 */
	}

	public Command getAutonomousCommand() {
		/* Run the path selected from the auto chooser */
		return autoChooser.getSelected();
	}
}
