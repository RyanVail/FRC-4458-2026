package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

public class WaitController extends Command {
    CommandGenericHID controller;
    XboxController.Button button;
    boolean finished;

    public WaitController(CommandGenericHID controller, XboxController.Button button) {
        this.controller = controller;
        this.button = button;

        controller.button(button.value).onTrue(Commands.runOnce(() -> finished = true));
    }

    @Override
    public void initialize()
    {
        finished = false;
    }

    @Override
    public boolean isFinished()
    {
        return finished;
    }
}
