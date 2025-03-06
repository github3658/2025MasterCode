package frc.robot.subsystems.hardware.Range3658;

import com.ctre.phoenix6.hardware.CANrange;

public class Range {
    private final CANrange _range;

    private double _detectionRange = 0.0;
    private int _sampleCount = 1;
    private int _actualCount = 0;

    Range(int deviceId, String canbus) {
        _range = new CANrange(deviceId, canbus);
    }

    CANrange getCaNrange() {
        return _range;
    }

    public Range setDetectionRange(double range) {
        _detectionRange = range;
        return this;
    }

    public double getDetectionRange() {
        return _detectionRange;
    }

    /** The number of times the sensor needs to see item in the range to consider it to be there. */
    public Range setSampleCount(int count) {
        _sampleCount = count;
        return this;
    }

    public int getSampleCount() {
        return _sampleCount;
    }

    public int getActualCount() {
        return _actualCount;
    }

    /** Will read the sensors distance. When in range it will increment the actualCount. If it counts up to the sample count it will flagged isInRange */
    public Range read() {
        if (_actualCount != _sampleCount) {
            if (isDetected()) {
                _actualCount++; 
            } else {
                _actualCount = 0;
            }
        }

        return this;
    }

    /** Resets the counter so it will start to read again. */
    public Range restart() {
        _actualCount = 0;
        return this;
    }

    public boolean isInRange() {
        return _actualCount == _sampleCount;
    }

    public double getDistance() {
        return _range.getDistance().getValueAsDouble();
    }

    public boolean isDetected() {
        return _range.getIsDetected().getValue();
    }
}
