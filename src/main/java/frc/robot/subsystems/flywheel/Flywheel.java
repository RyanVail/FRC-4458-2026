package frc.robot.subsystems.flywheel;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DoubleSupplier;
import frc.robot.PIDSupplier;
import frc.robot.Constants.FlyWheelConstants;

public class Flywheel extends SubsystemBase {
    FlywheelIO io;

    PIDSupplier leftPID = new PIDSupplier(LPREFIX + "leftPID", new PIDConstants(0.0));
    PIDSupplier rightPID = new PIDSupplier(LPREFIX + "rightPID", new PIDConstants(0.0));

    SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0, 0.00215);
    SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0, 0.00215);

    double setpoint = 0.0;

    /**
     * Supplies the distance from the robot to the target.
     */
    Supplier<Double> distance;

    public SysIdRoutine left = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism((Voltage v) -> {
            io.setLeftVoltage(v.magnitude());
        },
        log -> {
                // Record a frame for the shooter motor.
                log.motor("flywheel-left")
                    .voltage(
                        Units.Volt.of(io.getLeftVoltage()))
                    .angularPosition(Units.Radians.of(io.getLeftPosition()))
                    .angularVelocity(Units.RadiansPerSecond.of(io.getLeftVelocity() * 2 * Math.PI / 60));
        }
        , this));

    public SysIdRoutine right = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism((Voltage v) -> {
            io.setRightVoltage(v.magnitude());
        },
        log -> {
                // Record a frame for the shooter motor.
                log.motor("flywheel-right")
                    .voltage(
                        Units.Volt.of(io.getRightVoltage()))
                    .angularPosition(Units.Radians.of(io.getRightPosition()))
                    .angularVelocity(Units.RadiansPerSecond.of(io.getRightVoltage() * 2 * Math.PI / 60));
        }
        , this));

    boolean spinning = false;

    private static final InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();
    static {
        for (double[] entry : FlyWheelConstants.VEL_MAP) {
            velocityMap.put(entry[0], entry[1]);
        }
    };

    DoubleSupplier tmpVelocity = new DoubleSupplier(LPREFIX + "tmpVelocity", 0.0);

    private static final String LPREFIX = "/Subsystems/Flywheel/";

    public Flywheel(FlywheelIO io, Supplier<Double> distance) {
        this.io = io;
        this.distance = distance;
    }

    @Override
    public void periodic() {
        PIDController left = leftPID.get();
        PIDController right = rightPID.get();

        double target = getTargetVelocity();
        if (spinning && setpoint != target) {
            setpoint = target;
        } else if (!spinning) {
            setpoint = 0.0;
        }

        left.setSetpoint(setpoint);
        right.setSetpoint(setpoint);

        double leftVel = getLeftVelocity();
        double rightVel = getRightVelocity();

        double leftOutput = left.calculate(leftVel) + leftFF.calculate(setpoint);
        double rightOutput = right.calculate(rightVel) + rightFF.calculate(setpoint);

        // double leftOutput = tmpVelocity.get();
        // double rightOutput = tmpVelocity.get();

        setLeftVoltage(leftOutput);
        setRightVoltage(rightOutput);
        io.simulationPeriodic();

        Logger.recordOutput(LPREFIX + "Target", target);
        Logger.recordOutput(LPREFIX + "Distance", distance.get());
        Logger.recordOutput(LPREFIX + "Setpoint", setpoint);

        Logger.recordOutput(LPREFIX + "LeftVelocity", leftVel);
        Logger.recordOutput(LPREFIX + "LeftVoltage", getLeftVoltage());
        Logger.recordOutput(LPREFIX + "LeftOutput", leftOutput);

        Logger.recordOutput(LPREFIX + "RightVelocity", rightVel);
        Logger.recordOutput(LPREFIX + "RightVoltage", getRightVoltage());
        Logger.recordOutput(LPREFIX + "RightOutput", rightOutput);
    }

    private double getTargetVelocity() {
        // return velocityMap.get(distance.get());
        return tmpVelocity.get();
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
