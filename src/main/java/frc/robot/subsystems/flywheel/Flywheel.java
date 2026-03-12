package frc.robot.subsystems.flywheel;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DoubleSupplier;
import frc.robot.FFSupplier;
import frc.robot.PIDSupplier;
import frc.robot.SlewSupplier;
import frc.robot.Constants.FlyWheelConstants;

public class Flywheel extends SubsystemBase {
    FlywheelIO io;

    public enum State {
        Interp,
        Fixed,
    }

    PIDSupplier leftPID = new PIDSupplier(LPREFIX + "leftPID", new PIDConstants(0.004, 0.0, 0.0));
    PIDSupplier rightPID = new PIDSupplier(LPREFIX + "rightPID", new PIDConstants(0.002, 0.0, 0.0));

    PIDSupplier leftSlowPID = new PIDSupplier(LPREFIX + "leftSlowPID", new PIDConstants(3.125));
    PIDSupplier rightSlowPID = new PIDSupplier(LPREFIX + "rightSlowPID", new PIDConstants(5.0));

    FFSupplier leftFF = new FFSupplier(LPREFIX + "leftFF", new FFSupplier.Config(0, 0.001805));
    FFSupplier rightFF = new FFSupplier(LPREFIX + "rightFF", new FFSupplier.Config(0, 0.001805));

    SlewSupplier leftSlew = new SlewSupplier(LPREFIX + "leftSlew", new SlewSupplier.Config(64.0, -17.0));
    SlewSupplier rightSlew = new SlewSupplier(LPREFIX + "leftSlew", new SlewSupplier.Config(64.0, -17.0));

    DoubleSupplier pidMinDelta = new DoubleSupplier(LPREFIX + "pidMinDelta", 210);
    DoubleSupplier slowPidMinDelta = new DoubleSupplier(LPREFIX + "pidSlowMinDelta", 135);

    DoubleSupplier unjamVoltage = new DoubleSupplier(LPREFIX + "unjamVoltage", -10.0);

    double setpoint = 0.0;

    /**
     * Supplies the distance from the robot to the target.
     */
    Supplier<Double> distance;

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

    public Flywheel(FlywheelIO io, Supplier<Double> distance) {
        this.io = io;
        this.distance = distance;
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

            io.simulationPeriodic();
            return;
        }

        PIDController left = leftPID.get();
        PIDController right = rightPID.get();

        PIDController leftSlow = leftSlowPID.get();
        PIDController rightSlow = rightSlowPID.get();

        double target = getTargetVelocity() + nudge;
        setpoint = spinning ? target : 0.0;

        left.setSetpoint(setpoint);
        right.setSetpoint(setpoint);

        leftSlow.setSetpoint(setpoint);
        rightSlow.setSetpoint(setpoint);

        double leftVel = getLeftVelocity();
        double rightVel = -getRightVelocity();

        double leftDelta = setpoint - leftVel;
        double rightDelta = setpoint - rightVel;

        // Feedforward output.
        double leftOutput = leftFF.get().calculate(setpoint);
        double rightOutput = rightFF.get().calculate(setpoint);

        // Only apply PID if the vel delta is above the min.
        // if (leftDelta > pidMinDelta.get() && setpoint > 20.0) {
        //     leftOutput += left.calculate(leftVel);
        // } else {
        //     left.calculate(leftVel);
        // }

        // if (leftDelta < -slowPidMinDelta.get() && setpoint > 20.0) {
        //     leftOutput += leftSlow.calculate(leftVel);
        // } else {
        //     leftSlow.calculate(leftVel);
        // }

        // if (rightDelta > pidMinDelta.get() && setpoint > 20.0) {
        //     rightOutput += right.calculate(rightVel);
        // } else {
        //     right.calculate(rightVel);
        // }

        // if (rightDelta < -slowPidMinDelta.get() && setpoint > 20.0) {
        //     rightOutput += rightSlow.calculate(rightVel);
        // } else {
        //     rightSlow.calculate(rightVel);
        // }

        // Clamping voltage outputs.
        leftOutput = MathUtil.clamp(leftOutput, -12.0, 12.0);
        rightOutput = MathUtil.clamp(rightOutput, -12.0, 12.0);

        // Applying slew rate.
        leftOutput = leftSlew.get().calculate(leftOutput);
        rightOutput = rightSlew.get().calculate(rightOutput);

        setLeftVoltage(leftOutput);
        setRightVoltage(-leftOutput);

        io.simulationPeriodic();

        Logger.recordOutput(LPREFIX + "Spinning", spinning);
        Logger.recordOutput(LPREFIX + "Target", target);
        Logger.recordOutput(LPREFIX + "Distance", distance.get());
        Logger.recordOutput(LPREFIX + "Setpoint", setpoint);

        Logger.recordOutput(LPREFIX + "LeftDelta", leftDelta);
        Logger.recordOutput(LPREFIX + "LeftVelocity", leftVel);
        Logger.recordOutput(LPREFIX + "LeftOutput", leftOutput);

        Logger.recordOutput(LPREFIX + "RightDelta", rightDelta);
        Logger.recordOutput(LPREFIX + "RightVelocity", rightVel);
        Logger.recordOutput(LPREFIX + "RightOutput", rightOutput);

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

    private double getTargetVelocity() {
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

    public double getLeftVelocity() {
        return io.getLeftVelocity();
    }

    public double getRightVelocity() {
        return io.getRightVelocity();
    }

    public double getLeftVoltage() {
        return io.getLeftVoltage();
    }

    public double getRightVoltage() {
        return io.getRightVoltage();
    }
}
