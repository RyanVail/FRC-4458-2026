package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DoubleSupplier;

public class Hopper extends SubsystemBase {
    public enum State {
        // Not running.
        Idle,

        // Pushing fuel into shooter.
        Running,

        // Run in reverse.
        FixJam,
    };

    HopperIO io;

    State state = State.Idle;

    DoubleSupplier conveyorVoltage = new DoubleSupplier(LPREFIX + "conveyorVoltage", 12.0);
    DoubleSupplier shooterVoltage = new DoubleSupplier(LPREFIX + "shooterVoltage", 12.0);
    DoubleSupplier cooldownVoltage = new DoubleSupplier(LPREFIX + "cooldownVoltage", 2.0);

    DoubleSupplier cooldownDuration = new DoubleSupplier(LPREFIX + "cooldownDuration", 0.2);

    double cooldownEnd;

    private static final String LPREFIX = "/Subsystems/Hopper/";

    public Hopper(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        switch (state) {
            case FixJam:
                setConveyorVoltage(0);
                setShooterVoltage(-shooterVoltage.get());
                break;
            case Running:
                setConveyorVoltage(conveyorVoltage.get());
                if (onCooldown()) {
                    setShooterVoltage(cooldownVoltage.get());
                } else {
                    setShooterVoltage(shooterVoltage.get());
                }

                break;
            default:
                setConveyorVoltage(0.0);
                setShooterVoltage(0.0);
                break;
        }
    }

    public void startCooldown() {
        cooldownEnd = Timer.getFPGATimestamp() + cooldownDuration.get();
    }

    public boolean onCooldown() {
        return Timer.getFPGATimestamp() <= cooldownEnd;
    }

    public void setState(State state) {
        this.state = state;
    }

    public void setConveyorVoltage(double voltage) {
        io.setconveyorVoltage(voltage);
    }

    public void setShooterVoltage(double voltage) {
        io.setShooterVoltage(voltage);
    }
}
