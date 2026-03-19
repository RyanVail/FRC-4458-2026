package frc.robot.subsystems.flywheel;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.DoubleSupplier;
import frc.robot.FFSupplier;
import frc.robot.PIDSupplier;
import frc.robot.SlewSupplier;

public class Flywheel extends SubsystemBase {
    FlywheelIO io;

    public enum State {
        Interp,
        Fixed,
    }

    PIDSupplier leftPID = new PIDSupplier(LPREFIX + "leftPID", new PIDConstants(0.0035, 0.0, 0.0));
    PIDSupplier rightPID = new PIDSupplier(LPREFIX + "rightPID", new PIDConstants(0.0035, 0.0, 0.0));
    PIDSupplier middlePID = new PIDSupplier(LPREFIX + "middlePID", new PIDConstants(0.0035, 0.0, 0.0));

    PIDSupplier leftSlowPID = new PIDSupplier(LPREFIX + "leftSlowPID", new PIDConstants(3.125));
    PIDSupplier rightSlowPID = new PIDSupplier(LPREFIX + "rightSlowPID", new PIDConstants(5.0));
    PIDSupplier middleSlowPID = new PIDSupplier(LPREFIX + "middlePID", new PIDConstants(5.0));

    FFSupplier leftFF = new FFSupplier(LPREFIX + "leftFF", new FFSupplier.Config(0, 0.00269));
    FFSupplier rightFF = new FFSupplier(LPREFIX + "rightFF", new FFSupplier.Config(0, 0.00269));
    FFSupplier middleFF = new FFSupplier(LPREFIX + "middleFF", new FFSupplier.Config(0, 0.00269));

    SlewSupplier leftSlew = new SlewSupplier(LPREFIX + "leftSlew", new SlewSupplier.Config(64.0, -17.0));
    SlewSupplier rightSlew = new SlewSupplier(LPREFIX + "leftSlew", new SlewSupplier.Config(64.0, -17.0));
    SlewSupplier middleSlew = new SlewSupplier(LPREFIX + "middleSlew", new SlewSupplier.Config(64.0, -17.0));

    DoubleSupplier pidMinDelta = new DoubleSupplier(LPREFIX + "pidMinDelta", 155);
    DoubleSupplier slowPidMinDelta = new DoubleSupplier(LPREFIX + "pidSlowMinDelta", 135);

    DoubleSupplier unjamVoltage = new DoubleSupplier(LPREFIX + "unjamVoltage", -10.0);

    double setpoint = 0.0;

    /**
     * Supplies the distance from the robot to the target.
     */
    Supplier<Double> distance;
    Supplier<Pose2d> pose;

    boolean spinning = false;
    boolean unjam = false;

    boolean shot = false;

    /**
     * A nudge in RPMs that can be forced by the operator.
     */
    double nudge = 0.0;

    State state = State.Interp;

    private static final InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();
    static {
        for (double[] entry : FlyWheelConstants.VEL_MAP) {
            velocityMap.put(entry[0], entry[1]);
        }
    };

    DoubleSupplier tmpVelocity = new DoubleSupplier(LPREFIX + "tmpVelocity", 4500.0);

    public static final String LPREFIX = "/Subsystems/Flywheel/";

    public Flywheel(FlywheelIO io, Supplier<Double> distance, Supplier<Pose2d> pose) {
        this.io = io;
        this.distance = distance;
        this.pose = pose;

        Preferences.initBoolean(LPREFIX + "/useTmp", false);
    }

    public void addNudge(double nudge) {
        this.nudge += nudge;
    }

    @Override
    public void periodic() {
        if (unjam) {
            double v = unjamVoltage.get();
            setLeftVoltage(v);
            setRightVoltage(-v);
            setMiddleVoltage(-v);

            io.simulationPeriodic();
            return;
        }

        PIDController left = leftPID.get();
        PIDController right = rightPID.get();
        PIDController middle = middlePID.get();

        PIDController leftSlow = leftSlowPID.get();
        PIDController rightSlow = rightSlowPID.get();
        PIDController middleSlow = middleSlowPID.get();

        double target = getTargetVelocity() + nudge;
        setpoint = spinning ? target : 0.0;

        left.setSetpoint(setpoint);
        right.setSetpoint(setpoint);
        middle.setSetpoint(setpoint);
        
        leftSlow.setSetpoint(setpoint);
        rightSlow.setSetpoint(setpoint);
        middleSlow.setSetpoint(setpoint);

        double leftVel = getLeftVelocity();
        double rightVel = -getRightVelocity();
        double middleVel = -getMiddleVelocity();

        double leftDelta = setpoint - leftVel;
        double rightDelta = setpoint - rightVel;
        double middleDelta = setpoint - middleVel;

        // Feedforward output.
        double leftOutput = leftFF.get().calculate(setpoint);
        double rightOutput = rightFF.get().calculate(setpoint);
        double middleOutput = rightFF.get().calculate(setpoint);
        
        // Only apply PID if the vel delta is above the min.
        if (leftDelta > pidMinDelta.get() && setpoint > 20.0) {
            leftOutput += left.calculate(leftVel);
        } else {
            left.calculate(leftVel);
        }

        if (leftDelta < -slowPidMinDelta.get() && setpoint > 20.0) {
            leftOutput += leftSlow.calculate(leftVel);
        } else {
            leftSlow.calculate(leftVel);
        }

        if (rightDelta > pidMinDelta.get() && setpoint > 20.0) {
            rightOutput += right.calculate(rightVel);
        } else {
            right.calculate(rightVel);
        }

        if (rightDelta < -slowPidMinDelta.get() && setpoint > 20.0) {
            rightOutput += rightSlow.calculate(rightVel);
        } else {
            rightSlow.calculate(rightVel);
        }

        if (middleDelta > pidMinDelta.get() && setpoint > 20.0) {
            middleOutput += middle.calculate(middleVel);
        } else {
            middle.calculate(middleVel);
        }

        if (middleDelta < -slowPidMinDelta.get() && setpoint > 20.0) {
            middleOutput += middleSlow.calculate(middleVel);
        } else {
            middleSlow.calculate(middleVel);
        }

        // Clamping voltage outputs.
        leftOutput = MathUtil.clamp(leftOutput, -12.0, 12.0);
        rightOutput = MathUtil.clamp(rightOutput, -12.0, 12.0);
        middleOutput = MathUtil.clamp(middleOutput, -12.0, 12.0);

        // Applying slew rate.
        leftOutput = leftSlew.get().calculate(leftOutput);
        rightOutput = rightSlew.get().calculate(rightOutput);
        middleOutput = middleSlew.get().calculate(middleOutput);

        setLeftVoltage(leftOutput);
        setRightVoltage(-rightOutput);
        setMiddleVoltage(-middleOutput);
        io.simulationPeriodic();

        Logger.recordOutput(LPREFIX + "Spinning", spinning);
        Logger.recordOutput(LPREFIX + "Target", target);
        Logger.recordOutput(LPREFIX + "Distance", distance.get());
        Logger.recordOutput(LPREFIX + "Setpoint", setpoint);

        Logger.recordOutput(LPREFIX + "LeftDelta", leftDelta);
        Logger.recordOutput(LPREFIX + "LeftVelocity", leftVel);
        Logger.recordOutput(LPREFIX + "LeftOutput", leftOutput);
        Logger.recordOutput(LPREFIX + "LeftVoltage", getLeftVoltage());

        Logger.recordOutput(LPREFIX + "RightDelta", rightDelta);
        Logger.recordOutput(LPREFIX + "RightVelocity", rightVel);
        Logger.recordOutput(LPREFIX + "RightOutput", rightOutput);
        Logger.recordOutput(LPREFIX + "RightVoltage", getRightVoltage());

        Logger.recordOutput(LPREFIX + "MiddleDelta", middleDelta);
        Logger.recordOutput(LPREFIX + "MiddleVelocity", middleVel);
        Logger.recordOutput(LPREFIX + "MiddleOutput", middleOutput);
        Logger.recordOutput(LPREFIX + "MiddleVoltage", getMiddleVoltage());

        Logger.recordOutput(LPREFIX + "Nudge", nudge);
    }

    public void setState(State state) {
        this.state = state;
    }

    public void startUnjam() {
        unjam = true;
    }

    public void stopUnjam() {
        unjam = false;
    }

    private double  getTargetVelocity() {
        if(Preferences.getBoolean(LPREFIX + "/useTmp", false)) return tmpVelocity.get();
        if(pose.get().getX() > 5 && pose.get().getX() < 11) return 3500;
        return switch (state) {
            case Interp -> velocityMap.get(distance.get());
            case Fixed -> FlyWheelConstants.FIXED_VEL;
        };
    }

    public void toggle() {
        spinning = !spinning;
    }

    public void start() {
        spinning = true;
    }

    public void stop() {
        spinning = false;
    }

    public void setLeftVoltage(double voltage) {
        io.setLeftVoltage(voltage);
    }

    public void setRightVoltage(double voltage) {
        io.setRightVoltage(voltage);
    }

    public void setMiddleVoltage(double voltage) {
        io.setMiddleVoltage(voltage);
    }

    public double getLeftVelocity() {
        return io.getLeftVelocity();
    }

    public double getRightVelocity() {
        return io.getRightVelocity();
    }

    public double getMiddleVelocity() {
        return io.getMiddleVelocity();
    }

    public double getLeftVoltage() {
        return io.getLeftVoltage();
    }

    public double getRightVoltage() {
        return io.getRightVoltage();
    }

    public double getMiddleVoltage() {
        return io.getMiddleVoltage();
    }
}
