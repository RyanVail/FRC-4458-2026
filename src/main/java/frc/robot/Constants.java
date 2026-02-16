package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlignPose;
import frc.robot.control.BetterTrapezoidProfile;

public final class Constants {
    static private Map<String, Object> entries = new HashMap<>();

    public static void registerConstant(String key, Object value) {
        entries.put(key, value);
    }

    @SuppressWarnings("unchecked")
    public static void loadConstants() {
        File file = new File(Filesystem.getDeployDirectory() + "/constants.json");
        ObjectMapper mapper = new ObjectMapper();

        try {
            Map<String, Object> map = mapper.readValue(file, Map.class);
            for (String key : map.keySet()) {
                Object obj = map.get(key);
                if (obj instanceof Double) {
                    Preferences.setDouble(key, (Double)obj);
                } else if (obj instanceof Map) {
                    Map<String, Object> dict = (Map<String, Object>)obj;
                    if (dict.get("kP") != null) {
                        PIDConstants constants = new PIDConstants(
                            (Double)dict.get("kP"),
                            (Double)dict.get("kI"),
                            (Double)dict.get("kD")
                        );

                        PIDSupplier.loadConstants(key, constants);
                    }
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private static void updateConstant(String key) {
        Object entry = entries.get(key);
        if (entry instanceof Double) {
            entries.put(key, Preferences.getDouble(key, (Double)entry));
        } else if (entry instanceof PIDConstants) {
            entries.put(key, PIDSupplier.gatherConstants(key));
        }
    }

    private static void updateConstants() {
        for (String key : entries.keySet()) {
            updateConstant(key);
        }
    }

    public static void constructEntries() {
        Commands.print("writing to " + Filesystem.getDeployDirectory().toPath().toString()).schedule();
        File file = new File(Filesystem.getDeployDirectory() + "/constants.json");
        ObjectMapper mapper = new ObjectMapper();
        updateConstants();

        try {
            mapper.writerWithDefaultPrettyPrinter().writeValue(file, entries);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static final boolean TUNNING = true;
    public static final double LOOP_TIME = 0.02;

    public static final boolean ZERO_ENCODERS = false;

    public class InputConstants {
        public static final int OPERATOR_CONTROLLER_PORT = 0;
        public static final int DRIVE_CONTROLLER_PORT = 1;

        public static final double RUMBLE_VALUE = 0.7;
        public static final double RUMBLE_SECONDS = 0.3;

        public static final double TRIGGER_THRESHOLD = 0.4;
    }

    public class DriveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(14.5);
        public static final double DEADZONE = 0.08;

        public static final PPHolonomicDriveController PPDriveController = new PPHolonomicDriveController(
                new PIDConstants(1.72, 0.0, 0.0),
                new PIDConstants(0.82, 0.0, 0.0));
    }

    public class AutoAlignConstants {
        public static final PIDController X_CONTROLLER = new PIDController(5.2, 0.0, 0.0);
        public static final PIDController Y_CONTROLLER = new PIDController(5.2, 0.0, 0.0);
        public static final PIDController ANGLE_CONTROLLER = new PIDController(6.0, 0.0, 0.0);

        public static final BetterTrapezoidProfile X_PROFILE = new BetterTrapezoidProfile(
                new BetterTrapezoidProfile.Constraints(3.2, 1.4));
        public static final BetterTrapezoidProfile Y_PROFILE = new BetterTrapezoidProfile(
                new BetterTrapezoidProfile.Constraints(3.2, 1.4));

        // The distance past which auto alignment will not take place. This is meant to
        // prevent acidental auto aligns and auto aligns to incorrect positions like (0,
        // 0).
        public static final double MAX_DIST = Units.feetToMeters(10.0);

        public static final AlignPose.Constraints DEFAULT_CONSTRAINTS = new AlignPose.Constraints(
                Units.inchesToMeters(0.67),
                Units.degreesToRadians(0.5),
                new ChassisSpeeds(0.0, 0.0, 0.0));

        static {
            ANGLE_CONTROLLER.enableContinuousInput(0, Units.degreesToRadians(360.0));
        }
    }

    public class VisionConstants {
        record Camera(
            String name,
            Transform3d transform,
            int width,
            int height,
            Rotation2d fov,
            int fps,
            double avg_latency_ms,
            double latency_std_dev_ms
        ) {};

        public static final Camera CAMERAS[] = {};
        public static final double MAX_SECONDS = 0.8;
    }

    public class FieldConstants {
        public static final AprilTagFieldLayout LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeWelded);

        private static final Translation2d target = new Translation2d(4.5, 4.1);

        public static final Translation2d getTarget() {
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                return target;
            } else {
                return FlippingUtil.flipFieldPosition(target);
            }
        }
    }

    public class IntakeConstants {
        public static final int PORT = 16;
    }

    public class FlyWheelConstants {
        public static final int LEFT_PORT = 15;
        public static final int RIGHT_PORT = 14;
        public static final double MOI = 0.005;
        public static final double GEARING = 1.0;
        public static final double[] STD_DEVS = { 0.0 };

        public static final double[][] VEL_MAP = {
            { 0.0, 0.0 },
            { 5.0, 100.0 }
        };
    }

    public class HopperConstants {
        public static final int LEFT_PORT = 12;
        public static final int RIGHT_PORT = 13;
    }
}
