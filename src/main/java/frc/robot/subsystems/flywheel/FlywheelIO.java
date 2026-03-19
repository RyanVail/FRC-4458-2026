package frc.robot.subsystems.flywheel;

public interface FlywheelIO {
    public void setLeftVoltage(double voltage);
    public void setRightVoltage(double voltage);
    public void setMiddleVoltage(double voltage);

    public double getLeftVelocity();
    public double getRightVelocity();
    public double getMiddleVelocity();

    public double getLeftPosition();
    public double getRightPosition();
    public double getMiddlePosition();

    public double getLeftVoltage();
    public double getRightVoltage();
    public double getMiddleVoltage();

    public default void simulationPeriodic() {}
}
