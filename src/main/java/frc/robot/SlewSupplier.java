package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Preferences;

public final class SlewSupplier implements Supplier<SlewRateLimiter> {
    String key;
    Config lastConfig;
    SlewRateLimiter value;

    public final static class Config {
        final double pos;
        final double neg;

        public Config(double pos, double neg) {
            this.pos = pos;
            this.neg = neg;
        }
    }

    public SlewSupplier(String key, Config backup) {
        if (Constants.TUNNING) {
            this.key = key;

            if (!Preferences.containsKey(key + ".pos")) {
                loadConstants(key, backup);
            }

            Config config = gatherConstants(key);
            lastConfig = config;
            this.value = new SlewRateLimiter(config.pos, config.neg, 0.0);
        } else {
            this.value = new SlewRateLimiter(backup.pos, backup.neg, 0.0);
        }
    }

    private static Config gatherConstants(String key) {
        double s = Preferences.getDouble(key + ".pos", 0.0);
        double v = Preferences.getDouble(key + ".neg", 0.0);
        return new Config(s, v);
    }

    private static void loadConstants(String key, Config config) {
        Preferences.setDouble(key + ".pos", config.pos);
        Preferences.setDouble(key + ".neg", config.neg);
    }

    @Override
    public final SlewRateLimiter get() {
        if (Constants.TUNNING) {
            Config c = gatherConstants(key);
            if (this.lastConfig.neg != c.neg || this.lastConfig.pos != c.pos) {
                this.value = new SlewRateLimiter(c.pos, c.neg, this.value.lastValue());
            }

            this.lastConfig = c;
        }

        return this.value;
    }
}