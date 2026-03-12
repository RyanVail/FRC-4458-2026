package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.TargetLock;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.State;

/**
 * A command that starts shooting by starting the flywheel and hopper until
 * interrupted. When interrupted the flywheel and hopper will be stopped.
 */
public class Shoot extends Command {
    Drive drive;
    Flywheel flywheel;
    Hopper hopper;

    public Shoot(Drive drive, Flywheel flywheel, Hopper hopper) {
        addRequirements(drive, flywheel, hopper);
        this.drive = drive;
        this.flywheel = flywheel;
        this.hopper = hopper;
    }

    public void initialize() {
        drive.setTargetLock(TargetLock.Hub);
        hopper.setState(State.Running);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setTargetLock(TargetLock.None);
        hopper.setState(State.Idle);
    }
}