package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PIDSupplier;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    IntakeIO io;

    PIDSupplier rotPID = new PIDSupplier(LPREFIX + "rotPID", new PIDConstants(0.0));

    private static final String LPREFIX = "/Subsystems/Intake/";

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LPREFIX + "Velocity", getVelocity());
        Logger.recordOutput(LPREFIX + "Voltage", getVoltage());
        
        double rotOutput = rotPID.get().calculate(getRotPosition());
        io.setRotVoltage(rotOutput);

        Logger.recordOutput(LPREFIX + "RotPosition", getRotPosition());
        Logger.recordOutput(LPREFIX + "RotOutput", rotOutput);

        io.simulationPeriodic();
    }

    public void moveDown() {
        rotPID.get().setSetpoint(IntakeConstants.ROT_DOWN_POS);
    }

    public void moveUp() {
        rotPID.get().setSetpoint(IntakeConstants.ROT_UP_POS);
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
}
