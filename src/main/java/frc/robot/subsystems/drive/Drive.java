package frc.robot.subsystems.drive;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.commands.TeleopCommand;
import frc.robot.VisionManager;

public class Drive extends SubsystemBase {
    public class PoseSupplier implements Supplier<Pose2d> {
        public Drive drive;

        public PoseSupplier(Drive drive) {
            this.drive = drive;
        }

        public Pose2d get() {
            return drive.getPose();
        }
    }

    private DriveIO io;
    private Rotation2d gyroOffset;

    public static final String LPREFIX = "/Subsystems/Drive/";

    public Drive(DriveIO io) {
        this.io = io;
        this.io.resetPose(new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d()));
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LPREFIX + "Pose", io.getPose());
        Logger.recordOutput(LPREFIX + "FlippedPose", FlippingUtil.flipFieldPose(io.getPose()));
        Logger.recordOutput(LPREFIX + "SwerveStates", io.getSwerveStates());

        io.periodic();

        List<EstimatedRobotPose> poses = VisionManager.getEstimatedPoses();
        for (int i = 0; i < poses.size(); i++) {
            if (poses.get(i) == null)
                continue;

            Logger.recordOutput(LPREFIX + "VisionPose" + i, poses.get(i).estimatedPose);
        }

        io.addVisionEstimations(poses);
    }

    /**
     * @return Instance of TeleopCommand
     */
    public Command getTeleopCommand(CommandGenericHID controller) {
        return new TeleopCommand(this, controller);
    }

    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    public Supplier<Pose2d> getPoseSupplier() {
        return new PoseSupplier(this);
    }

    public Pose2d getPose() {
        return io.getPose();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return io.getRobotRelativeSpeeds();
    }

    public void driveRobotRelative(double x, double y, double omega) {
        this.driveRobotRelative(new ChassisSpeeds(x, y, omega));
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.io.drive(speeds);
    }

    public void driveGyroRelative(double x, double y, double omega) {
        this.driveGyroRelative(new ChassisSpeeds(x, y, omega));
    }

    public void driveGyroRelative(ChassisSpeeds speeds) {
        this.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getGyroRotation()));
    }

    public void driveVisionRelative(double x, double y, double omega) {
        this.driveVisionRelative(new ChassisSpeeds(x, y, omega));
    }

    public void driveVisionRelative(ChassisSpeeds speeds) {
        this.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getPose().getRotation()));
    }

    public void setGryoOffset(Rotation2d rotation) {
        this.gyroOffset = rotation;
    }

    public void resetGyroOffset() {
        this.gyroOffset = this.io.getGyroRotation();
    }

    public Rotation2d getGyroRotation() {
        return this.io.getGyroRotation().minus(gyroOffset);
    }

    public ChassisSpeeds getRobotVelocity() {
        return io.getRobotVelocity();
    }

    public void stop() {
        this.driveRobotRelative(new ChassisSpeeds());
    }
}
