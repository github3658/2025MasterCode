package frc.robot.subsystems.hardware.Range3658;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;

public class RangeBuilder {
    private final CANrangeConfiguration _config = new CANrangeConfiguration();
    private final ProximityParamsConfigs _proxParams = new ProximityParamsConfigs();
    private final FovParamsConfigs _fovParams = new FovParamsConfigs();

    public RangeBuilder() { }

    /** Sets the threshold for object detection. */
    public RangeBuilder setProximityThreshold(double threshold) {
        _proxParams.withProximityThreshold(threshold);
        return this;
    }

    /** How far above and below the threshold the distance needs to be to trigger undetected and detected, respectively. This is used to prevent bouncing between the detected and undetected states for objects on the threshold.
        If the threshold is set to 0.1 meters, and the hysteresis is 0.01 meters, then an object needs to be within 0.09 meters to be detected. After the object is first detected, the distance then needs to exceed 0.11 meters to become undetected again. */
    public RangeBuilder setProximityHysteresis(double hysteresis) {
        _proxParams.withProximityHysteresis(hysteresis);
        return this;
    }

    /** Specifies the target range of the Field of View in the X direction. This is the full range of the FOV. */
    public RangeBuilder setFOVRangeX(double x) {
        _fovParams.withFOVRangeX(x);
        return this;
    }

    /** Specifies the target center of the Field of View in the Y direction. */
    public RangeBuilder setFOVRangeY(double y) {
        _fovParams.withFOVCenterY(y);
        return this;
    }

    /** Will apply range configurations and initialize the range sensor. */
    public Range build(int deviceId, String canbus) {
        Range range = new Range(deviceId, canbus);

        range.getCaNrange().getConfigurator().apply(
            _config.withProximityParams(_proxParams).withFovParams(_fovParams)
        );

        return range;
    }
}
