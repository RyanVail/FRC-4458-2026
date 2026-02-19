package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DoubleSupplier;

public class Hopper extends SubsystemBase {
    HopperIO io;
    boolean spinning = false;

    DoubleSupplier conveyorVoltage = new DoubleSupplier(LPREFIX + "conveyorVoltage", 3.0);
    DoubleSupplier shooterVoltage = new DoubleSupplier(LPREFIX + "shooterVoltage", 3.0);

    private static final String LPREFIX = "/Subsystems/Hopper/";

    public Hopper(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        setconveyorVoltage(spinning ? conveyorVoltage.get() : 10.0);
        setShooterVoltage(spinning ? shooterVoltage.get() : 12.0);

        Logger.recordOutput(LPREFIX + "Spinning", spinning);
    }

    public void start() {
        spinning = true;
    }

    public void stop() {
        spinning = false;
    }

    public void setconveyorVoltage(double voltage) {
        io.setconveyorVoltage(voltage);
    }

    public void setShooterVoltage(double voltage) {
        io.setShooterVoltage(voltage);
    }
}
