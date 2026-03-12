package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSwerve;
import frc.robot.subsystems.drive.Drive.TargetLock;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSpark;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.intake.Intake.State;

public class RobotContainer {
    CommandGenericHID operatorHID;
    CommandGenericHID driverHID;

    Drive drive;
    Intake intake;
    Flywheel flywheel;
    Hopper hopper;

    public RobotContainer() {
        operatorHID = new CommandGenericHID(InputConstants.OPERATOR_CONTROLLER_PORT);
        driverHID = new CommandGenericHID(InputConstants.DRIVE_CONTROLLER_PORT);

        Supplier<Double> shoot_distance = () -> drive.getPose().getTranslation().getDistance(
                FieldConstants.getHubPos());

        if (Robot.isSimulation()) {
            drive = new Drive(new DriveIOSwerve());
            intake = new Intake(new IntakeIOSim());
            flywheel = new Flywheel(new FlywheelIOSim(), shoot_distance);
            hopper = new Hopper(new HopperIOSim());
        } else {
            drive = new Drive(new DriveIOSwerve());
            intake = new Intake(new IntakeIOSpark());
            flywheel = new Flywheel(new FlywheelIOSpark(), shoot_distance);
            hopper = new Hopper(new HopperIOSpark());
        }

        VisionManager.initialize();
        configureAuto();

        configureBindings();
        drive.resetGyroOffset();
    }

    private void configureAuto() {
        NamedCommands.registerCommand(
                "StopDrive",
                Commands.runOnce(() -> drive.stop()));

        NamedCommands.registerCommand(
                "KeepStopDrive",
                Commands.run(() -> drive.stop()));

        NamedCommands.registerCommand(
                "StartFlywheel",
                Commands.runOnce(() -> flywheel.start()));

        NamedCommands.registerCommand(
                "IntakeIntake",
                Commands.runOnce(() -> intake.setState(State.Intaking)));

        NamedCommands.registerCommand(
                "IntakeIdle",
                Commands.runOnce(() -> intake.setState(State.Idle)));

        NamedCommands.registerCommand(
                "StartHopper",
                Commands.runOnce(() -> hopper.setState(Hopper.State.Idle)));

        NamedCommands.registerCommand(
                "StopHopper",
                Commands.runOnce(() -> hopper.setState(Hopper.State.Running)));

        NamedCommands.registerCommand(
                "LockOnHub",
                Commands.runOnce(() -> drive.setTargetLock(TargetLock.Hub)));

        NamedCommands.registerCommand(
                "Unlock",
                Commands.runOnce(() -> drive.setTargetLock(TargetLock.None)));

        NamedCommands.registerCommand(
                "StartShooting",
                Commands.runOnce(() -> {
                    intake.setState(State.Oscillating);
                    hopper.setState(Hopper.State.Running);
                }));

        NamedCommands.registerCommand(
                "StopShooting",
                Commands.runOnce(() -> {
                    intake.setState(State.Idle);
                    hopper.setState(Hopper.State.Idle);
                }));

        NamedCommands.registerCommand(
                "EjectPreload",
                Commands.sequence(
                        Commands.runOnce(() -> intake.setState(State.PreloadEject)),
                        Commands.waitSeconds(1),
                        Commands.runOnce(() -> intake.setState(State.Idle))));

        NamedCommands.registerCommand("Shoot", new Shoot(flywheel, hopper));

        AutoManager.configureAutos(drive);
    }

    /**
     * This method will be used to configure controls
     */
    private void configureBindings() {
        SmartDashboard.putData("Reset Gyro", Commands.runOnce(() -> {
            drive.resetGyroOffset();
        }));

        SmartDashboard.putData("Command scheduler", CommandScheduler.getInstance());

        SmartDashboard.putData("Flywheel override", Commands.runOnce(() -> {
            flywheel.setState(Flywheel.State.Fixed);
        }));

        SmartDashboard.putData("Flywheel interp", Commands.runOnce(() -> {
            flywheel.setState(Flywheel.State.Interp);
        }));

        driverHID.button(XboxController.Button.kStart.value).onTrue(Commands.runOnce(() -> {
            drive.resetGyroOffset();
        }));

        operatorHID.button(XboxController.Button.kA.value).onTrue(Commands.runOnce(() -> {
            flywheel.toggle();
        }));

        operatorHID.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.2).onTrue(Commands.runOnce(() -> {
            hopper.setState(Hopper.State.Running);
            intake.setState(State.Oscillating);
        }));

        operatorHID.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.2).onFalse(Commands.runOnce(() -> {
            hopper.setState(Hopper.State.Idle);
            intake.setState(State.Idle);
        }));

        operatorHID.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.2).onTrue(Commands.runOnce(() -> {
            drive.setTargetLock(TargetLock.Hub);
        }));

        operatorHID.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.2).onFalse(Commands.runOnce(() -> {
            drive.setTargetLock(TargetLock.None);
        }));

        operatorHID.button(XboxController.Button.kRightBumper.value).onTrue(Commands.runOnce(() -> {
            flywheel.startUnjam();
            hopper.setState(Hopper.State.FixJam);
        }));

        operatorHID.button(XboxController.Button.kRightBumper.value).onFalse(Commands.runOnce(() -> {
            flywheel.stopUnjam();
            hopper.setState(Hopper.State.Idle);
        }));

        driverHID.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.2).onTrue(Commands.runOnce(() -> {
            intake.setState(State.Intaking);
        }));

        driverHID.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.2).onFalse(Commands.runOnce(() -> {
            intake.setState(State.Idle);
        }));

        driverHID.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.2).onTrue(Commands.runOnce(() -> {
            intake.setState(State.Bump);
        })).onFalse(Commands.runOnce(() -> {
            intake.setState(State.Idle);
        }));

        operatorHID.povUp().onTrue(Commands.runOnce(() -> {
            flywheel.addNudge(FlyWheelConstants.NUDGE_INC);
        }));

        operatorHID.povDown().onTrue(Commands.runOnce(() -> {
            flywheel.addNudge(-FlyWheelConstants.NUDGE_INC);
        }));

        drive.setDefaultCommand(
                drive.getTeleopCommand(
                        (Robot.isReal()) ? driverHID : operatorHID));
    }

    public void teleopInit() {
        flywheel.start();
        drive.setTargetLock(TargetLock.None);
        hopper.setState(Hopper.State.Idle);
        intake.setState(State.Idle);
    }
}
