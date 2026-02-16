package frc.robot.subsystems.flywheel;

public interface FlywheelIO {
    public void setVoltage(double voltage);

    public double getVelocity();

    public double getVoltage();

    public default void simulationPeriodic() {}
}
