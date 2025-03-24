package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger {
    private static final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private static final NetworkTable table = nt.getTable("RoboBeavers");

    private static StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
    private static StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();
        
    public static void initLogger() {
        SmartDashboard.putString("RoboBeavers", "Best Team");
    }
    public static void writeBoolean(String key, Boolean value) {
        table.putValue(key, NetworkTableValue.makeBoolean(value));
    }

    public static void writeDouble(String key, Double value) {
        table.putValue(key, NetworkTableValue.makeDouble(value));
    }

    public static void writeString(String key, String value) {
        table.putValue(key, NetworkTableValue.makeString(value));
    }

    public static void writeInteger(String key, Integer value) {
        table.putValue(key, NetworkTableValue.makeInteger(value));
    }

    public static void write2DPos(Pose2d pos) {
        publisher.set(pos);
    }
    public static void write2DPosArray(Pose2d[] pos) {
        arrayPublisher.set(pos);
    }
}

