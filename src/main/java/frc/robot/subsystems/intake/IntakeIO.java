package frc.robot.subsystems.intake;

public interface IntakeIO {
    public void setVoltage(double voltage);

    public void setRotVoltage(double voltage);

    public double getRotPosition();

    public double getVelocity();

    public double getVoltage();

    public double getPosition();

    public double getRotVelocity();

    public default void simulationPeriodic() {}
}
