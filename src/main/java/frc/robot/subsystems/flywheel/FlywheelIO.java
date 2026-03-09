package frc.robot.subsystems.flywheel;

public interface FlywheelIO {
    public void setLeftVelocitySetpoint(double voltage);
    public void setRightVelocitySetpoint(double voltage);

    public double getLeftVelocity();
    public double getRightVelocity();

    public double getLeftPosition();
    public double getRightPosition();

    public default void periodic() {}
    public default void simulationPeriodic() {}
}
