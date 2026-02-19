package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.DoubleSupplier;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.drive.Drive;

public class TeleopCommand extends Command {
    Drive drive;
    CommandGenericHID controller;

    DoubleSupplier axisControlPow = new DoubleSupplier("AxisControlPow", 3.0);
    DoubleSupplier deadzone = new DoubleSupplier("ControlDeadzone", 0.08);

    SlewRateLimiter xl = new SlewRateLimiter(5);
    SlewRateLimiter yl = new SlewRateLimiter(5);

    private Notifier rumble;

    public TeleopCommand(Drive drive, CommandGenericHID controller) {
        super.addRequirements(drive);
        this.drive = drive;
        this.controller = controller;
    }

    private void startRumble() {
        if (this.rumble != null)
            this.rumble.close();

        controller.setRumble(RumbleType.kBothRumble, InputConstants.RUMBLE_VALUE);
        this.rumble = new Notifier(
                () -> {
                    controller.setRumble(RumbleType.kBothRumble, 0.0);
                });

        this.rumble.startSingle(InputConstants.RUMBLE_SECONDS);
    }

    public void initialize() {
        startRumble();
    }

    private double processAxis(double axis) {
        boolean neg = axis <= 0;
        axis = Math.abs(axis);

        // Apply deadzone.
        if (axis <= deadzone.get())
            return 0;

        // Make the drive speed exponential.
        axis = (neg ? -1 : 1) * Math.pow(axis, axisControlPow.get());

        return axis;
    }

    @Override
    public void execute() {
        if (DriverStation.isAutonomous()) {
            this.drive.stop();
            return;
        }

        double x = processAxis(controller.getRawAxis(1));
        x = xl.calculate(x);
        double y = processAxis(controller.getRawAxis(0));
        y = yl.calculate(y);
        double yaw = processAxis(-controller.getRawAxis(4));

        drive.driveRobotRelative(
                -x * DriveConstants.MAX_SPEED,
                -y * DriveConstants.MAX_SPEED,
                yaw * DriveConstants.MAX_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        startRumble();
    }
}