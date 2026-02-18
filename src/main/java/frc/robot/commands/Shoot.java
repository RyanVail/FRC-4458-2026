package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hopper.Hopper;

/**
 * A command that starts shooting by starting the flywheel and hopper until
 * interrupted. When interrupted the flywheel and hopper will be stopped.
 */
public class Shoot extends Command {
    Flywheel flywheel;
    Hopper hopper;

    public Shoot(Flywheel flywheel, Hopper hopper) {
        addRequirements(flywheel, hopper);
        this.flywheel = flywheel;
        this.hopper = hopper;
    }

    public void initialize() {
        flywheel.start();
        hopper.start();
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        hopper.stop();
    }
}