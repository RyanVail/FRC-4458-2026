package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.TargetLock;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.State;
import frc.robot.subsystems.intake.Intake;

/**
 * A command that starts shooting by starting the flywheel and hopper until
 * interrupted. When interrupted the flywheel and hopper will be stopped.
 */
public class Shoot extends Command {
    Drive drive;
    Hopper hopper;
    Intake intake;

    public Shoot(Drive drive, Hopper hopper, Intake intake) {
        addRequirements(drive, hopper);
        this.drive = drive;
        this.hopper = hopper;
        this.intake = intake;
    }

    public void initialize() {
        drive.setTargetLock(TargetLock.Hub);
        intake.setState(Intake.State.Oscillating);
        hopper.setState(State.Running);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setTargetLock(TargetLock.None);
        intake.setState(Intake.State.Idle);
        hopper.setState(State.Idle);
    }
}