package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Preferences;

public class DoubleSupplier implements Supplier<Double> {
    String key;
    double value;

    public DoubleSupplier(String key, double backup) {
        if (Constants.TUNNING) {
            this.key = key;

            // When constants get updated this will load in the correct value.
            Constants.registerConstant(key, backup);

            if (!Preferences.containsKey(key)) {
                Preferences.setDouble(key, backup);
            }
        } else {
            this.value = Preferences.getDouble(key, backup);
        }
    }

    public Double get() {
        if (Constants.TUNNING) {
            return Preferences.getDouble(key, 0.0);
        } else {
            return this.value;
        }
    }
}