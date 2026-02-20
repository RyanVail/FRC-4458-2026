package frc.robot.subsystems.flywheel;

public interface FlywheelIO {
    public void setLeftVoltage(double voltage);
    public void setRightVoltage(double voltage);

    public double getLeftVelocity();
    public double getRightVelocity();

    public double getLeftPosition();
    public double getRightPosition();

    public double getLeftVoltage();
    public double getRightVoltage();

    public default void simulationPeriodic() {}
}
