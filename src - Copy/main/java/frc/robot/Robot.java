// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Robot extends LoggedRobot {
    @SuppressWarnings("unused")
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        if (Robot.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
            Logger.addDataReceiver(new NT4Publisher());
        } else if (Robot.isSimulation()) {
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.recordMetadata("BuildConstants/GitSHA",
                BuildConstants.GIT_SHA);
        Logger.recordMetadata("BuildConstants/GitBranch",
                BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("BuildConstants/BuildDate",
                BuildConstants.BUILD_DATE);
        Logger.recordMetadata("BuildConstants/BuildUnixTimestamp",
                String.valueOf(BuildConstants.BUILD_UNIX_TIME));

        Logger.start();

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        AutoManager.start();
    }


    @Override
    public void teleopInit() {
        AutoManager.cancel();

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }
}
