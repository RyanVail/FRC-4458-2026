package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Preferences;

public final class FFSupplier implements Supplier<SimpleMotorFeedforward> {
    String key;
    SimpleMotorFeedforward value;

    public final static class Config {
        final double kS;
        final double kV;

        public Config(double s, double v) {
            this.kS = s;
            this.kV = v;
        }
    }

    public FFSupplier(String key, Config backup) {
        if (Constants.TUNNING) {
            this.key = key;

            if (!Preferences.containsKey(key + ".s")) {
                loadConstants(key, backup);
            }
        }

        Config constants = gatherConstants(key);
        this.value = new SimpleMotorFeedforward(constants.kS, constants.kV);
    }

    private static final Config gatherConstants(String key) {
        double s = Preferences.getDouble(key + ".s", 0.0);
        double v = Preferences.getDouble(key + ".v", 0.0);
        return new Config(s, v);
    }

    private static final void loadConstants(String key, Config config) {
        Preferences.setDouble(key + ".s", config.kS);
        Preferences.setDouble(key + ".v", config.kV);
    }

    @Override
    public final SimpleMotorFeedforward get() {
        if (Constants.TUNNING) {
            Config c = gatherConstants(key);
            if (this.value.getKs() != c.kS || this.value.getKv() != c.kV) {
                this.value.setKs(c.kS);
                this.value.setKv(c.kV);
            }
        }

        return this.value;
    }
}