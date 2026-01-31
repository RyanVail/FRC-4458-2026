package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class AutoManager {
    private static SendableChooser<PathPlannerAuto> chooser = new SendableChooser<>();
    private static PathPlannerAuto auto;

    private static boolean inited;

    private static final String[] autos = {};

    public static void configureAutos(Drive drive) {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                drive::getPose,
                drive::resetPose,
                drive::getRobotRelativeSpeeds,
                (speeds) -> drive.driveRobotRelative(speeds),
                DriveConstants.PPDriveController,
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return (alliance.isPresent())
                            ? alliance.get() == DriverStation.Alliance.Red
                            : false;
                },
                drive);

        try {
            for (String str : autos) {
                chooser.addOption(str, new PathPlannerAuto(str));
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        SmartDashboard.putData("Auto", chooser);
    }

    public static void start() {
        AutoManager.inited = true;

        AutoManager.auto = chooser.getSelected();
        if (AutoManager.auto != null)
            AutoManager.auto.schedule();
    }

    public static void cancel() {
        if (AutoManager.inited && AutoManager.auto != null)
            AutoManager.auto.cancel();
    }
}
