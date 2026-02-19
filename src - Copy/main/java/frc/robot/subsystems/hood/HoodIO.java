package frc.robot.subsystems.hood;

public interface HoodIO {
    public void setSetpoint(double setpoint);

    public double getLeftPosition();
    public double getRightPosition();

    public default void periodic() {}

    public default void simulationPeriodic() {}
}
