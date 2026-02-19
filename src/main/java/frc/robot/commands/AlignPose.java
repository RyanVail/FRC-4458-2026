package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.PIDSupplier;
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

    private static final String LPREFIX = "/Commands/AlignPose/";

    static final PIDSupplier xPID = new PIDSupplier(LPREFIX + "xPID", new PIDConstants(0));
    static final PIDSupplier yPID = new PIDSupplier(LPREFIX + "yPID", new PIDConstants(5.2, 0.0, 0.0));
    static final PIDSupplier anglePID = new PIDSupplier(LPREFIX + "anglePID", new PIDConstants(6.0, 0.0, 0.0));

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

        
        anglePID.get().enableContinuousInput(0, Units.degreesToRadians(360.0));
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

        xPID.get().reset();
        yPID.get().reset();
        anglePID.get().reset();

        anglePID.get().setSetpoint(target.pose.getRotation().getRadians());
        anglePID.get().setTolerance(target.constraints.rot_dist);
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

        xPID.get().setSetpoint(lastXState.position);
        yPID.get().setSetpoint(lastYState.position);

        Logger.recordOutput("AutoAlignState", new Pose2d(lastXState.position, lastYState.position, new Rotation2d()));

        // TODO: It might be better for this to be gyro relative once the gyro offset during auto starts working.
        drive.driveVisionRelative(
                xPID.get().calculate(pose.getX()),
                yPID.get().calculate(pose.getY()),
                anglePID.get().calculate(pose.getRotation().getRadians()));
    }

    @Override
    public boolean isFinished() {
        if (this.target == null)
            return false;

        return drive.getPose().getTranslation()
                .getDistance(target.pose.getTranslation()) <= this.target.constraints.dist
                && anglePID.get().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.finished = true;
        this.drive.stop();
    }
}