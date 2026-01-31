package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSwerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSpark;

public class RobotContainer {
    CommandGenericHID operatorHID;
    CommandGenericHID driverHID;

    Drive drive;
    Intake intake;

    public RobotContainer() {
        Constants.loadConstants();

        operatorHID = new CommandGenericHID(InputConstants.OPERATOR_CONTROLLER_PORT);

        // For simulations it's easier to have one controller.
        if (Robot.isReal()) {
            driverHID = new CommandGenericHID(InputConstants.DRIVE_CONTROLLER_PORT);
        }

        if (Robot.isSimulation()) {
            drive = new Drive(new DriveIOSwerve());
        } else {
            drive = new Drive(new DriveIOSwerve());
            intake = new Intake(new IntakeIOSpark());
        }

        // VisionManager.initialize(drive);
        // VisionManager.defaultCameras();
        // configureAuto();

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

        operatorHID.button(XboxController.Button.kA.value).onTrue(Commands.runOnce(() -> {
            Constants.constructEntries();
        }));

        drive.setDefaultCommand(
                drive.getTeleopCommand(
                        (Robot.isReal()) ? driverHID : operatorHID));
    }
}
