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
    public enum State {
        // Held high to keep over bump.
        Idle,

        // Held down to intake fuel.
        Intaking,

        // Oscillating to help feed fuel into the shooter.
        Oscillating,

        // Slowly crunch inward to feed fuel into the shooter.
        Crunching,
    }

    IntakeIO io;
    State state = State.Idle;
    double rotSetpoint;

    double shootStart;

    PIDSupplier rotPID = new PIDSupplier(LPREFIX + "rotPID", new PIDConstants(0.05));
    ArmFeedforward rotFF = new ArmFeedforward(0.00002, 0.000185, 0.00002);
    TrapezoidProfile rotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            20.0,
            5.0));

    DoubleSupplier ossilateScale = new DoubleSupplier(LPREFIX + "ossilate", 20.0);
    DoubleSupplier crunchSpeed = new DoubleSupplier(LPREFIX + "crunchSpeed", 4.0);

    DoubleSupplier upPos = new DoubleSupplier(LPREFIX + "upPos", 30.0);
    DoubleSupplier idlePos = new DoubleSupplier(LPREFIX + "rotIdle", 60.0);
    DoubleSupplier intakePos = new DoubleSupplier(LPREFIX + "intakePos", 80.0);

    DoubleSupplier voltage = new DoubleSupplier(LPREFIX + "voltage", 10.0);
    DoubleSupplier shootingVoltage = new DoubleSupplier(LPREFIX + "shootingVoltage", 2.0);

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

        rotSetpoint = nextRotSetpoint();
        TrapezoidProfile.State state = rotProfile.calculate(
                Constants.LOOP_TIME,
                new TrapezoidProfile.State(rotPosition, rotVelocity),
                new TrapezoidProfile.State(rotSetpoint, 0.0));

        rotPID.get().setSetpoint(rotSetpoint);

        double rotOutput = rotPID.get().calculate(state.position);

        rotOutput += rotFF.calculate(
                Units.degreesToRadians(rotPosition - 90.0),
                Units.degreesToRadians(state.velocity));

        io.setRotVoltage(rotOutput);

        Logger.recordOutput(LPREFIX + "RotSetpoint", rotSetpoint);
        Logger.recordOutput(LPREFIX + "RotPosition", rotPosition);
        Logger.recordOutput(LPREFIX + "RotOutput", rotOutput);

        double targetVoltage = nextTargetVoltage();

        Logger.recordOutput(LPREFIX + "TargetVoltage", targetVoltage);
        setVoltage(targetVoltage);

        io.simulationPeriodic();
    }

    private double nextRotSetpoint() {
        double time = Timer.getFPGATimestamp() - shootStart;
        return switch (state) {
            case Intaking -> intakePos.get();
            case Oscillating ->
                intakePos.get() - ((Math.sin(time * 9.0) + 1.0) * 0.5 * ossilateScale.get());
            case Crunching -> Math.max(intakePos.get() - time * crunchSpeed.get(), upPos.get());
            default -> idlePos.get();
        };
    }

    private double nextTargetVoltage() {
        return switch (state) {
            case Intaking -> voltage.get();
            case Oscillating -> shootingVoltage.get();
            case Crunching -> shootingVoltage.get();
            default -> 0.0;
        };
    }

    public void start() {
        state = State.Intaking;
    }

    public void stop() {
        state = State.Idle;
    }

    public void startShooting() {
        shootStart = Timer.getFPGATimestamp();
        state = State.Oscillating;
    }

    public void stopShooting() {
        state = State.Idle;
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
