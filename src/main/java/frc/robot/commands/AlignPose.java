package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.control.BetterTrapezoidProfile.State;
import frc.robot.subsystems.drive.Drive;

public class AlignPose extends Command {
    Drive drive;
    private Target target;
    AlignCamera camera;

    State lastXState;
    State lastYState;

    State XStateSetpoint;
    State YStateSetpoint;

    boolean finished;

    public enum AlignCamera {
        NONE,
        ALL,
        BACK,
        FRONT,
    };

    public static class Constraints {
        // The translational distance tolerance in meters.
        double dist;

        // The rotational distance tolerance in radians.
        double rot_dist;

        // The target velocity at the pose. Only the translational values of this are
        // used.
        ChassisSpeeds vel;

        public Constraints(double dist, double rot_dist, ChassisSpeeds vel) {
            this.dist = dist;
            this.rot_dist = rot_dist;
            this.vel = vel;
        }
    }

    public static class Target {
        private Pose2d pose;
        private Constraints constraints;

        public Target(Pose2d pose) {
            this.pose = pose;
            this.constraints = AutoAlignConstants.DEFAULT_CONSTRAINTS;
        }

        public Target(Pose2d pose, Constraints constraints) {
            this.pose = pose;
            this.constraints = constraints;
        }
    };

    public AlignPose(Drive drive, Target target, AlignCamera camera) {
        addRequirements(drive);
        this.drive = drive;
        this.camera = camera;

        if (target != null)
            setTarget(target);
    }

    public void setTarget(Target target) {
        this.target = target;

        Pose2d start = drive.getPose();
        ChassisSpeeds speeds = drive.getRobotVelocity();

        Logger.recordOutput("AligningTo", target.pose);

        if (!withinStartingDistance(start, target.pose)) {
            Commands.print("Auto align dist too far. Start: " + start + " target: " + this.target.pose).schedule();
            this.target = null;
            super.cancel();
            return;
        }

        XStateSetpoint = new State(target.pose.getX(), target.constraints.vel.vxMetersPerSecond);
        YStateSetpoint = new State(target.pose.getY(), target.constraints.vel.vyMetersPerSecond);

        lastXState = new State(start.getX(), speeds.vxMetersPerSecond);
        lastYState = new State(start.getY(), speeds.vyMetersPerSecond);

        AutoAlignConstants.X_CONTROLLER.reset();
        AutoAlignConstants.Y_CONTROLLER.reset();
        AutoAlignConstants.ANGLE_CONTROLLER.reset();

        AutoAlignConstants.ANGLE_CONTROLLER.setSetpoint(target.pose.getRotation().getRadians());
        AutoAlignConstants.ANGLE_CONTROLLER.setTolerance(target.constraints.rot_dist);
    }

    /**
     * @param start  The starting position.
     * @param target The position to align to.
     * @return True if the start and target poses are close enough to allow for
     *         aligning, false otherwise.
     */
    public boolean withinStartingDistance(Pose2d start, Pose2d target) {
        return start.getTranslation().getDistance(target.getTranslation()) <= AutoAlignConstants.MAX_DIST;
    }

    public void initialize() {
        if (finished && this.target != null)
            this.setTarget(this.target);

        this.finished = false;
    }

    @Override
    public void execute() {
        if (this.target == null || lastXState == null || lastYState == null)
            return;

        Pose2d pose = drive.getPose();

        lastXState = AutoAlignConstants.X_PROFILE.calculate(Constants.LOOP_TIME, XStateSetpoint, lastXState);
        lastYState = AutoAlignConstants.Y_PROFILE.calculate(Constants.LOOP_TIME, YStateSetpoint, lastYState);

        AutoAlignConstants.X_CONTROLLER.setSetpoint(lastXState.position);
        AutoAlignConstants.Y_CONTROLLER.setSetpoint(lastYState.position);

        Logger.recordOutput("AutoAlignState", new Pose2d(lastXState.position, lastYState.position, new Rotation2d()));

        // TODO: It might be better for this to be gyro relative once the gyro offset during auto starts working.
        drive.driveVisionRelative(
                AutoAlignConstants.X_CONTROLLER.calculate(pose.getX()),
                AutoAlignConstants.Y_CONTROLLER.calculate(pose.getY()),
                AutoAlignConstants.ANGLE_CONTROLLER.calculate(pose.getRotation().getRadians()));
    }

    @Override
    public boolean isFinished() {
        if (this.target == null)
            return false;

        return drive.getPose().getTranslation()
                .getDistance(target.pose.getTranslation()) <= this.target.constraints.dist
                && AutoAlignConstants.ANGLE_CONTROLLER.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.finished = true;
        this.drive.stop();
    }
}