// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
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
import frc.robot.Autos.Drive1;
import frc.robot.Enums.ElevatorHight;
import frc.robot.Enums.FishHookState;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {

	Elevator m_elevator = new Elevator();
	Fish_Hook m_fish_hook = new Fish_Hook();
	//Climber m_climber = new Climber();

	private double MaxSpeed =
			TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
	private double MaxAngularRate =
			RotationsPerSecond.of(0.75)
					.in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final SwerveRequest.FieldCentric drive =
			new SwerveRequest.FieldCentric()
					.withDeadband(MaxSpeed * 0.05)
					.withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
					.withDriveRequestType(
							DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	private final SwerveRequest.RobotCentric forwardStraight =
			new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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

		// // Set the logger to log to the first flashdrive plugged in
		// SignalLogger.setPath("/media/sda1/");

		// // Explicitly start the logger
		// SignalLogger.start();

		// // Explicitly stop logging
		// // If the user does not call stop(), then it's possible to lose the last few seconds of data
		// SignalLogger.stop();
	}

	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(
						() ->
								drive
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

		// Zero the elevator and fishhook when disabled
		//	RobotModeTriggers.disabled().onFalse(m_elevator.runOnce(() -> m_elevator.zeroElevator()));
		//	RobotModeTriggers.disabled().onFalse(m_fish_hook.runOnce(() -> m_fish_hook.zeroFishhook()));

		// TODO Find button for: driverController.a().toggleOnTrue(drivetrain.applyRequest(() ->
		// brake));
		// joystick.b().whileTrue(drivetrain.applyRequest(() ->
		//     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
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

		// reset the field-centric heading on start button press
		driverController.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		drivetrain.registerTelemetry(logger::telemeterize);

		// Driver Buttons

		// Idle to bottom
		// m_elevator.setDefaultCommand(new CMD_ElevatorState(ElevatorHight.State.BOTTOM, m_elevator));

		// Elevator positions
		driverController.a().onTrue(new CMD_ElevatorState(ElevatorHight.State.L1, m_elevator));
		driverController.b().onTrue(new CMD_ElevatorState(ElevatorHight.State.L2, m_elevator));
		driverController.x().onTrue(new CMD_ElevatorState(ElevatorHight.State.L3, m_elevator));
		driverController.y().onTrue(new CMD_ElevatorState(ElevatorHight.State.L4, m_elevator));

		// TODO right bumper for right side of reef alignment

		// driverController.rightTrigger().whileTrue(new ReefRight(drivetrain));
		// TODO left bumper for left side of reef alignment

		// driverController.leftTrigger().whileTrue(new ReefLeft(drivetrain));

		driverController
				.button(7)
				.onTrue(new CMD_ElevatorState(ElevatorHight.State.BOTTOM, m_elevator));

		driverController
					.rightBumper()
					.whileTrue(
						m_elevator
							.run(() -> m_elevator.elevator(0.5))
							.withInterruptBehavior(InterruptionBehavior.kCancelSelf));
		
		driverController
					.leftBumper()
					.whileTrue(
						m_elevator
							.run(() -> m_elevator.elevator(-0.5))
							.withInterruptBehavior(InterruptionBehavior.kCancelSelf));

		m_elevator.setDefaultCommand(
				m_elevator.run(
						() -> {
								m_elevator.elevator(0);						
						}));

		// Co-Driver Buttons

		// Idle fishHook in
		// m_fish_hook.setDefaultCommand(new CMD_FishHookState(FishHookState.State.IDLE, m_fish_hook));
		m_fish_hook.setDefaultCommand(
				m_fish_hook.run(
						() -> {
							// if (!m_fish_hook.autoIntakeRunning()) {
							// 	m_fish_hook.coral(0);
							// }
							m_fish_hook.algae(0);
							// m_fish_hook.tilt(0.0);
							m_fish_hook.tilt(
									codriverJoystick.getRawAxis(Joystick.AxisType.kY.value)
											* 0.1); // get the y axis of the joystick and multiply by 0.1 to get the speed
							// new CMD_FishHookState(FishHookState.State.IDLE, m_fish_hook).execute();
						}));

		// idle climber in
		// m_climber.setDefaultCommand(new CMD_ClimberState(ClimberState.State.IN, m_climber));

		coDriverController
				.button(1 /*Trigger*/)
				.whileTrue(
						m_fish_hook
								.run(() -> m_fish_hook.coral(-0.5))
								.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)); // run coral motors

		coDriverController
				.button(2 /*Bottom Face button*/)
				.whileTrue(new Intake(m_elevator, m_fish_hook)); // intake coral

		coDriverController
				.button(3 /*Left face button*/)
				.whileTrue(
						m_fish_hook
								.run(() -> m_fish_hook.algae(-0.5))
								.withInterruptBehavior(InterruptionBehavior.kCancelSelf)); // intake algae

		coDriverController
				.button(4 /*Right face button*/)
				.whileTrue(
						m_fish_hook
								.run(() -> m_fish_hook.algae(0.5))
								.withInterruptBehavior(InterruptionBehavior.kCancelSelf)); // outtake algae

		coDriverController
				.button(5)
				.onTrue(
						new CMD_FishHookState(FishHookState.Angle.IDLE, m_fish_hook)); // move fishhook to idle

		coDriverController
				.button(6)
				.onTrue(
						new CMD_FishHookState(
								FishHookState.Angle.Intake, m_fish_hook)); // move fishhook to Intake/L1, L2, L3

		coDriverController
				.button(7)
				.onTrue(new CMD_FishHookState(FishHookState.Angle.L4, m_fish_hook)); // move fishhook to L4

		coDriverController
				.button(8)
				.onTrue(
						new CMD_FishHookState(
								FishHookState.Angle.Algae, m_fish_hook)); // move fishhook to Algea

		// coDriverController
		// 		.button(9)
		// 		.onTrue(new CMD_ClimberState(ClimberState.State.IN, m_climber)); // move climber to IN

		// coDriverController
		// 		.button(10)
		// 		.onTrue(new CMD_ClimberState(ClimberState.State.OUT, m_climber)); // move climber to OUT

		// coDriverController
		// 		.button(11)
		// 		.onTrue(
		// 				m_climber
		// 						.run(() -> m_climber.climber(0.5))
		// 						.withInterruptBehavior(InterruptionBehavior.kCancelSelf)); // move climber up

		// coDriverController
		// 		.button(12)
		// 		.onTrue(
		// 				m_climber
		// 						.run(() -> m_climber.climber(-0.5))
		// 						.withInterruptBehavior(InterruptionBehavior.kCancelSelf)); // move climber down
	}

	public Command getAutonomousCommand() {
		/* Run the path selected from the auto chooser */
		return autoChooser.getSelected();
	}
}
