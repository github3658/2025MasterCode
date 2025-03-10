package frc.robot;

public class Config {
    public static final String kCanbus = "3658";
    public static final String kLimelight = "limelight";
    public static final int kPigeon = 13;
    public static final int kCandle = 60;
    public static final int kLEDCount = 68;
    //region Swerve
    public static final int kBackLeftDriveMotor = 1;
    public static final int kBackLeftAngleMotor = 2;
    public static final int kBackRightDriveMotor = 3;
    public static final int kBackRightAngleMotor = 4;
    public static final int kFrontLeftDriveMotor = 5;
    public static final int kFrontLeftAngleMotor = 6;
    public static final int kFrontRightDriveMotor = 7;
    public static final int kFrontRightAngleMotor = 8;
    public static final int kBackLeftCanCoder = 10;
    public static final int kBackRightCanCoder = 11;
    public static final int kFrontLeftCanCoder = 12;
    public static final int kFrontRightCanCoder = 13;
    //endregion

    //region Elevator
    public static final int kLeftElevatorMotor = 41;
    public static final int kRightElevatorMotor = 42;
    public static final int kElevatorLimitSwitch = 0;
    //endregion

    //region Climb
    public static final int kLiftMotor = 51;
    //endregion

    //region Endofactor
    public static final int kPivotMotorId = 31;
    public static final int kDeliveryMotorId = 32;
    public static final int kDeliveryRangeId = 61;
    public static final int kPivotEncoderId = 33; 
    public static final int kAlgeaLimitSwitch = 1;
    //endregion
}
