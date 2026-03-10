package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AlignPose;
import frc.robot.control.BetterTrapezoidProfile;

public final class Constants {
    public static final boolean TUNNING = true;
    public static final double LOOP_TIME = 0.02;

    public static final boolean ZERO_ENCODERS = false;

    public static final class InputConstants {
        public static final int OPERATOR_CONTROLLER_PORT = 0;
        public static final int DRIVE_CONTROLLER_PORT = 1;

        public static final double RUMBLE_VALUE = 0.7;
        public static final double RUMBLE_SECONDS = 0.3;

        public static final double TRIGGER_THRESHOLD = 0.4;
    }

    public static final class DriveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(14.5);
        public static final double MAX_TURN_SPEED = 4.4196;
        public static final double DEADZONE = 0.08;

        public static final PPHolonomicDriveController PPDriveController = new PPHolonomicDriveController(
                new PIDConstants(1.72, 0.0, 0.0),
                new PIDConstants(0.82, 0.0, 0.0));
    }

    public static final class AutoAlignConstants {
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
    }

    public static final class VisionConstants {
        record Camera(
                String name,
                Transform3d transform,
                int width,
                int height,
                Rotation2d fov,
                int fps,
                double avg_latency_ms,
                double latency_std_dev_ms) {
        };

        public static final Camera CAMERAS[] = {
                new Camera("Front",
                        new Transform3d(
                                Units.inchesToMeters(8.0), // 0
                                Units.inchesToMeters(0.0), // 7.5
                                Units.inchesToMeters(26), // 28
                                new Rotation3d(0, Units.degreesToRadians(-55), 0)),
                        0,
                        0,
                        Rotation2d.kZero,
                        0,
                        0,
                        0)
        };
        public static final double MAX_SECONDS = 0.8;

        /**
         * The method to use when estimating a pose from april tags.
         */
        public static final Method METHOD = Method.COPROC_MULTI_TAG;

        public enum Method {
            COPROC_MULTI_TAG,
            AVERAGE_BEST,
            LEAST_AMBIGUOUS,
            CLOSEST_HEIGHT,
        }
    }

    public static final class FieldConstants {
        public static final AprilTagFieldLayout LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltWelded);

        private static final Translation2d TARGET = LAYOUT.getTagPose(10).get().toPose2d().getTranslation()
                .plus((LAYOUT.getTagPose(4).get().toPose2d().getTranslation())).div(2);

        /**
         * Gets the target hub position auto flipped based on team.
         */
        public static final Translation2d getHubPos() {
            if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
                return TARGET;
            } else {
                return FlippingUtil.flipFieldPosition(TARGET);
            }
        }

        private static final double[][] TIME_MAP_DATA = {
            { 0.0, 1.0 },
            { 1.0, 4.0 },
            { 2.0, 7.0 },
        };

        public static final InterpolatingDoubleTreeMap TIME_MAP = new InterpolatingDoubleTreeMap();
        static {
            for (double[] entry : TIME_MAP_DATA) {
                TIME_MAP.put(entry[0], entry[1]);
            }
        }
    }

    public static final class IntakeConstants {
        public static final int PORT = 12;

        /**
         * The port of the motor that controls the rotation of the intake.
         */
        public static final int ROT_PORT = 8;
    }

    public static final class FlyWheelConstants {
        public static final int LEFT_PORT = 14;
        public static final int RIGHT_PORT = 10;
        public static final double MOI = 0.005;
        public static final double GEARING = 1.0;
        public static final double[] STD_DEVS = { 0.0 };

        public static final double[][] VEL_MAP = {
                { 3.4, 3400.0 },
        };

        public static final double P = 0.005;
        public static final double FF_V = 0.02;
    }

    public static final class HoodConstants {
        public static final int LEFT_CHANNEL = 0;
        public static final int RIGHT_CHANNEL = 1;

        /**
         * The max length of the servo in mm.
         */
        public static final double LENGTH = 100.0;

        /**
         * The speed of ths servo in mm per second.
         */
        public static final double SPEED = 22.5;

        public static final double[][] POS_MAP = {
                { 0.0, 0.0 },
                { 3.4, 50.0 },
        };
    }

    public static final class HopperConstants {
        public static final int CONVEYOR_PORT = 9;
        public static final int SHOOTER_PORT = 11;
    }
}
