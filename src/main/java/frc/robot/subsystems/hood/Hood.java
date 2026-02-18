package frc.robot.subsystems.hood;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DoubleSupplier;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
    HoodIO io;
    double setpoint = 0.0;

    /**
     * Supplies the distance from the robot to the target.
     */
    Supplier<Double> distance;

    private static final InterpolatingDoubleTreeMap posMap = new InterpolatingDoubleTreeMap();
    static {
        for (double[] entry : HoodConstants.POS_MAP) {
            posMap.put(entry[0], entry[1]);
        }
    };

    DoubleSupplier tmpSetpoint = new DoubleSupplier("tmpSetpoint", 0.0);

    private static final String LPREFIX = "/Subsystems/Hood/";

    public Hood(HoodIO io, Supplier<Double> distance) {
        this.io = io;
        this.distance = distance;
    }

    @Override
    public void periodic() {
        io.periodic();
        io.simulationPeriodic();
    
        // TODO: TMP use the distance.
        if (tmpSetpoint.get() != setpoint) {
            setSetopint(tmpSetpoint.get());
        }

        Logger.recordOutput(LPREFIX + "leftPosition", getLeftPosition());
        Logger.recordOutput(LPREFIX + "rightPosition", getRightPosition());
        Logger.recordOutput(LPREFIX + "setpoint", setpoint);
    }

    public void setSetopint(double setpoint) {
        this.setpoint = setpoint;
        io.setSetpoint(setpoint);
    }

    public double getLeftPosition() {
        return io.getLeftPosition();
    }

    public double getRightPosition() {
        return io.getRightPosition();
    }
}
