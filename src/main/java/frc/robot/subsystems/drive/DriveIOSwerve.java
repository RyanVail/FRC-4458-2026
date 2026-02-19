package frc.robot.subsystems.drive;

import java.io.File;
import java.io.IOException;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PnpResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class DriveIOSwerve implements DriveIO {
    private SwerveDrive swerveDrive;

    public DriveIOSwerve() {
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(
                    Units.feetToMeters(DriveConstants.MAX_SPEED));
        } catch (IOException e) {
            e.printStackTrace();
        }

        swerveDrive.setCosineCompensator(false);
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setOdometryPeriod(Constants.LOOP_TIME);
    }

    @Override
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    @Override
    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        swerveDrive.drive(speeds);
    }

    @Override
    public Rotation2d getRotation() {
        return swerveDrive.getPose().getRotation();
    }

    @Override
    public SwerveModuleState[] getSwerveStates() {
        return swerveDrive.getStates();
    }

    @Override
    public void addVisionEstimations(List<EstimatedRobotPose> poses) {
        for (EstimatedRobotPose pose : poses) {
            if (pose != null && pose.estimatedPose != null) {
                swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
                        pose.estimatedPose.toPose2d(),
                        pose.timestampSeconds);
            }
        }
    }

    @Override
    public void addVisionEstimationsPnp(List<PnpResult> poses) {
        for (PnpResult result : poses) {
            if (result != null && result.best != null) {
                swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
                        new Pose2d(
                                result.best.getTranslation().toTranslation2d(),
                                result.best.getRotation().toRotation2d()),
                        Timer.getFPGATimestamp());
            }
        }
    }

    @Override
    public Rotation2d getGyroRotation() {
        return swerveDrive.getGyroRotation3d().toRotation2d();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }
}