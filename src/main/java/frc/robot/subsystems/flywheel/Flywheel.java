package frc.robot.subsystems.flywheel;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DoubleSupplier;
import frc.robot.PIDSupplier;

public class Flywheel extends SubsystemBase {
    FlywheelIO io;
    PIDSupplier pid = new PIDSupplier(LPREFIX + "pid", new PIDConstants(0.0));
    DoubleSupplier tmpSpeed = new DoubleSupplier(LPREFIX + "tmpSpeed", 2.0);

    /**
     * Supplies the distance from the robot to the target.
     */
    Supplier<Double> distance;

    boolean spinning = false;

    private static final String LPREFIX = "/Subsystems/Flywheel/";

    public Flywheel(FlywheelIO io, Supplier<Double> distance) {
        this.io = io;
        this.distance = distance;
    }

    @Override
    public void periodic() {
        PIDController controller = pid.get();

        double target = getTargetVelocity();
        if (spinning && controller.getSetpoint() != target) {
            controller.setSetpoint(target);
        } else if (!spinning) {
            controller.setSetpoint(0.0);
        }

        double vel = getVelocity();
        double output = controller.calculate(vel);

        SmartDashboard.putData(LPREFIX + "PID", controller);
        Logger.recordOutput(LPREFIX + "Velocity", vel);
        Logger.recordOutput(LPREFIX + "Voltage", getVoltage());
        Logger.recordOutput(LPREFIX + "Target", controller.getSetpoint());
        Logger.recordOutput(LPREFIX + "Output", output);

        setVoltage(output);
        io.simulationPeriodic();
    }

    // TODO: This should actually be using the distance.
    private double getTargetVelocity() {
        return tmpSpeed.get();
    }

    public void start() {
        spinning = true;
    }

    public void stop() {
        spinning = false;
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public double getVelocity() {
        return io.getVelocity();
    }

    public double getVoltage() {
        return io.getVoltage();
    }
}
