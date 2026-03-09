package frc.robot.subsystems.flywheel;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DoubleSupplier;
import frc.robot.Constants.FlyWheelConstants;

public class Flywheel extends SubsystemBase {
    FlywheelIO io;

    /**
     * Supplies the distance from the robot to the target.
     */
    Supplier<Double> distance;

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
        double target = getTargetVelocity();

        double setpoint = spinning ? target : 0.0;
        io.setLeftVelocitySetpoint(setpoint);
        io.setRightVelocitySetpoint(setpoint);
        io.simulationPeriodic();

        Logger.recordOutput(LPREFIX + "Target", target);
        Logger.recordOutput(LPREFIX + "Distance", distance.get());
        Logger.recordOutput(LPREFIX + "Setpoint", setpoint);

        Logger.recordOutput(LPREFIX + "LeftVelocity", getLeftVelocity());
        Logger.recordOutput(LPREFIX + "RightVelocity", getRightVelocity());
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

    public double getLeftVelocity() {
        return io.getLeftVelocity();
    }

    public double getRightVelocity() {
        return io.getRightVelocity();
    }
}
