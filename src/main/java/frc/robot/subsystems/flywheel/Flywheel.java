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

    PIDSupplier leftPID = new PIDSupplier(LPREFIX + "leftPID", new PIDConstants(0.002, 0.0, 0.0));
    PIDSupplier rightPID = new PIDSupplier(LPREFIX + "rightPID", new PIDConstants(0.002, 0.0, 0.0));

    PIDSupplier leftSlowPID = new PIDSupplier(LPREFIX + "leftSlowPID", new PIDConstants(5.0));
    PIDSupplier rightSlowPID = new PIDSupplier(LPREFIX + "rightSlowPID", new PIDConstants(5.0));

    FFSupplier leftFF = new FFSupplier(LPREFIX + "leftFF", new FFSupplier.Config(0, 0.001805));
    FFSupplier rightFF = new FFSupplier(LPREFIX + "rightFF", new FFSupplier.Config(0, 0.001805));

    SlewSupplier leftSlew = new SlewSupplier(LPREFIX + "leftSlew", new SlewSupplier.Config(64.0, -17.0));
    SlewSupplier rightSlew = new SlewSupplier(LPREFIX + "leftSlew", new SlewSupplier.Config(64.0, -17.0));

    DoubleSupplier pidMinDelta = new DoubleSupplier(LPREFIX + "pidMinDelta", 200);
    DoubleSupplier slowPidMinDelta = new DoubleSupplier(LPREFIX + "pidSlowMinDelta", 108);

    DoubleSupplier unjamVoltage = new DoubleSupplier(LPREFIX + "unjamVoltage", -10.0);

    double setpoint = 0.0;

    /**
     * Supplies the distance from the robot to the target.
     */
    Supplier<Double> distance;

    boolean spinning = false;
    boolean unjam = false;

    boolean shot = false;
    double lastLeftDelta;
    double lastRightDelta;

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

    @Override
    public void periodic() {
        if (unjam) {
            double v = unjamVoltage.get();
            setLeftVoltage(v);
            setRightVoltage(-v);
            return;
        }

        PIDController left = leftPID.get();
        PIDController right = rightPID.get();

        PIDController leftSlow = leftSlowPID.get();
        PIDController rightSlow = rightSlowPID.get();

        double target = getTargetVelocity();
        if (spinning && setpoint != target) {
            setpoint = target;
        } else if (!spinning) {
            setpoint = 0.0;
        }

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

        // Clamping voltage outputs.
        leftOutput = MathUtil.clamp(leftOutput, -12.0, 12.0);
        rightOutput = MathUtil.clamp(rightOutput, -12.0, 12.0);

        // Applying slew rate.
        leftOutput = leftSlew.get().calculate(leftOutput);
        rightOutput = rightSlew.get().calculate(rightOutput);

        setLeftVoltage(leftOutput);
        setRightVoltage(-rightOutput);

        Logger.recordOutput(LPREFIX + "Target", target);
        Logger.recordOutput(LPREFIX + "Distance", distance.get());
        Logger.recordOutput(LPREFIX + "Setpoint", setpoint);

        Logger.recordOutput(LPREFIX + "LeftDelta", leftDelta);
        Logger.recordOutput(LPREFIX + "LeftVelocity", leftVel);
        Logger.recordOutput(LPREFIX + "LeftOutput", leftOutput);

        Logger.recordOutput(LPREFIX + "RightDelta", rightDelta);
        Logger.recordOutput(LPREFIX + "RightVelocity", rightVel);
        Logger.recordOutput(LPREFIX + "RightOutput", rightOutput);

        checkShot(leftDelta, rightDelta);
    }

    public void startUnjam() {
        unjam = true;
    }

    public void stopUnjam() {
        unjam = false;
    }

    public Supplier<Boolean> getShot() {
        return () -> shot;
    }

    private void checkShot(double leftDelta, double rightDelta) {
        boolean leftShot = setpoint > 20.0
                && lastLeftDelta < -3.0
                && leftDelta > 3.0;

        boolean rightShot = setpoint > 20.0
                && lastRightDelta < -3.0
                && rightDelta > 3.0;

        Logger.recordOutput(LPREFIX + "leftShot", leftShot);
        Logger.recordOutput(LPREFIX + "rightShot", rightShot);

        lastLeftDelta = leftDelta;
        lastRightDelta = rightDelta;
        shot = leftShot || rightShot;
    }

    private double getTargetVelocity() {
        return velocityMap.get(distance.get());
        // return tmpVelocity.get();
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
