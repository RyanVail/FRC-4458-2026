package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSwerve;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSpark;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOSpark;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSpark;

public class RobotContainer {
    CommandGenericHID operatorHID;
    CommandGenericHID driverHID;

    Drive drive;
    Intake intake;
    Flywheel flywheel;
    Hopper hopper;
    Hood hood;

    public RobotContainer() {
        Constants.loadConstants();

        operatorHID = new CommandGenericHID(InputConstants.OPERATOR_CONTROLLER_PORT);

        // For simulations it's easier to have one controller.
        if (Robot.isReal()) {
            driverHID = new CommandGenericHID(InputConstants.DRIVE_CONTROLLER_PORT);
        }

        Supplier<Double> shoot_distance = () -> drive.getPose().getTranslation().getDistance(
                FieldConstants.getTarget());

        if (Robot.isSimulation()) {
            drive = new Drive(new DriveIOSwerve());
            flywheel = new Flywheel(new FlywheelIOSim(), shoot_distance);
            hopper = new Hopper(new HopperIOSim());
        } else {
            drive = new Drive(new DriveIOSwerve());
            intake = new Intake(new IntakeIOSpark());
            flywheel = new Flywheel(new FlywheelIOSpark(), shoot_distance);
            hopper = new Hopper(new HopperIOSpark());
            hood = new Hood(new HoodIOSpark(), shoot_distance);
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
                Commands.run(() -> flywheel.start()));

        NamedCommands.registerCommand(
                "StartIntake",
                Commands.run(() -> intake.start()));

        NamedCommands.registerCommand(
                "StopIntake",
                Commands.run(() -> intake.stop()));

        NamedCommands.registerCommand(
                "StartHopper",
                Commands.run(() -> hopper.start()));

        NamedCommands.registerCommand(
                "StopHopper",
                Commands.run(() -> hopper.stop()));

        NamedCommands.registerCommand(
                "ToggleTargetLock",
                Commands.run(() -> drive.toggleTargetLock()));

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

        if (Constants.TUNNING) {
            SmartDashboard.putData("Construct Constants", Commands.runOnce(() -> {
                Constants.constructEntries();
            }).ignoringDisable(true));
        }

        if (Robot.isReal()) {
            driverHID.button(XboxController.Button.kStart.value).onTrue(Commands.runOnce(() -> {
                drive.resetGyroOffset();
            }));
        }

        operatorHID.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.2).onTrue(Commands.runOnce(() -> {
            flywheel.start();
        }));

        operatorHID.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.2).onFalse(Commands.runOnce(() -> {
            flywheel.stop();
        }));

        operatorHID.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.2).onTrue(Commands.runOnce(() -> {
            hopper.start();
        }));

        operatorHID.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.2).onFalse(Commands.runOnce(() -> {
            hopper.stop();
        }));

        operatorHID.button(XboxController.Button.kY.value).onTrue(Commands.runOnce(() -> {
            drive.toggleTargetLock();
        }));

        operatorHID.button(XboxController.Button.kB.value).onTrue(Commands.runOnce(() -> {
            intake.start();
        }));

        operatorHID.button(XboxController.Button.kB.value).onFalse(Commands.runOnce(() -> {
            intake.stop();
        }));

        drive.setDefaultCommand(
                drive.getTeleopCommand(
                        (Robot.isReal()) ? driverHID : operatorHID));
    }
}
