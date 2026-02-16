package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DoubleSupplier;

public class Hopper extends SubsystemBase {
    HopperIO io;
    boolean spinning = false;

    DoubleSupplier voltage = new DoubleSupplier(LPREFIX + "voltage", 3.0);

    private static final String LPREFIX = "/Subsystems/Hopper/";

    public Hopper(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        setVoltage(spinning ? voltage.get() : 0.0);

        Logger.recordOutput(LPREFIX + "Spinning", spinning);
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
}
