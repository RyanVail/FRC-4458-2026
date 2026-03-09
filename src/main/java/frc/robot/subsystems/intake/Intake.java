package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DoubleSupplier;
import frc.robot.PIDSupplier;

public class Intake extends SubsystemBase {
    IntakeIO io;

    PIDSupplier rotPID = new PIDSupplier(LPREFIX + "rotPID", new PIDConstants(0.05));
    ArmFeedforward rotFF = new ArmFeedforward(0.00002, 0.000185, 0.00002);
    TrapezoidProfile rotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        20.0,
        5.0
    ));

    DoubleSupplier rotUpPos = new DoubleSupplier(LPREFIX + "rotUpPos", 0.0);
    DoubleSupplier rotDownPos = new DoubleSupplier(LPREFIX + "rotDownPos", 80.0);
    DoubleSupplier ossilateScale = new DoubleSupplier(LPREFIX + "Ossilate", 20.0);

    DoubleSupplier voltage = new DoubleSupplier(LPREFIX + "voltage", 10.0);
    DoubleSupplier shootingVoltage = new DoubleSupplier(LPREFIX + "shootingVoltage", 2.0);

    double rotSetpoint;
    boolean spinning = false;
    boolean shooting = false;
    boolean down = false;

    private static final String LPREFIX = "/Subsystems/Intake/";

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LPREFIX + "Velocity", getVelocity());
        Logger.recordOutput(LPREFIX + "Voltage", getVoltage());

        double rotPosition = getRotPosition();
        double rotVelocity = getRotVelocity();

        TrapezoidProfile.State state = rotProfile.calculate(
            Constants.LOOP_TIME,
            new TrapezoidProfile.State(rotPosition, rotVelocity),
            new TrapezoidProfile.State(rotSetpoint, 0.0)
        );

        if (shooting) {
            double x = (Math.sin(Timer.getFPGATimestamp() * 9.0) + 1.0) * 0.5;
            rotSetpoint = rotDownPos.get() - (x * ossilateScale.get());
        } else if (down) {
            rotSetpoint = rotDownPos.get();
        } else {
            rotSetpoint = rotUpPos.get();
        }

        rotPID.get().setSetpoint(rotSetpoint);
        double rotOutput = rotPID.get().calculate(rotPosition);

        rotOutput += rotFF.calculate(
            Units.degreesToRadians(rotPosition - 90.0),
            state.velocity
        );

        io.setRotVoltage(rotOutput);

        Logger.recordOutput(LPREFIX + "RotSetpoint", rotSetpoint);
        Logger.recordOutput(LPREFIX + "RotPosition", rotPosition);
        Logger.recordOutput(LPREFIX + "RotOutput", rotOutput);
 
        double targetVoltage = 0.0;
        if (spinning) {
            targetVoltage = voltage.get();
        } else if (shooting) {
            targetVoltage = shootingVoltage.get();
        }

        Logger.recordOutput(LPREFIX + "TargetVoltage", targetVoltage);
        setVoltage(targetVoltage);

        io.simulationPeriodic();
    }

    public void start() {
        spinning = true;
    }

    public void stop() {
        spinning = false;
    }

    public void moveDown() {
        down = true;
    }

    public void moveUp() {
        down = false;
    }

    public void startShooting() {
        shooting = true;
    }

    public void stopShooting() {
        shooting = false;
    }

    public void toggleDown() {
        if (down) {
            moveUp();
        } else {
            moveDown();
        }
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

    public double getPosition() {
        return io.getPosition();
    }

    public double getRotPosition() {
        return io.getRotPosition();
    }

    public double getRotVelocity() {
        return io.getRotVelocity();
    }
}
