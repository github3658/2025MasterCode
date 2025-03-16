package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

import com.ctre.phoenix.led.CANdle;
// com.ctre.phoenix not resolving is probably a quirk with a VS Code extension. The code builds fine.
// This no longer seems to happen in 2025.

public class LEDSubsystem extends SubsystemBase {
    /**
     * An enum of configured color presets for the LED subsystem. Includes:
     * <p> Green
     * <p> Yellow
     * <p> Blue
     * <p> Orange
     * <p> Red
     * <p> White
     * <p> Magenta
     */
    public enum Color {
        Green(0, 255, 0),
        Yellow(255, 192, 0),
        Blue(0, 0, 255),
        Orange(255, 96, 0),
        Red(255, 0, 0),
        White(255, 255, 255),
        Magenta(255, 0, 255),
        Alliance(0, 0, 0);
        public int r, g, b;
        Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }
    private Color e_ColorTarget;

    /**
     * An enum of configured patterns for the LED subsystem. Includes:
     * <p> Solid
     */
    public enum Pattern {
        Solid,
    }
    private Pattern e_PatternTarget;

    /* CONSTANTS (prefix: c) */
    private final int c_CANDleID = Config.kCandle;
    private final int c_LEDCount = Config.kLEDCount;

    private int i_Timer;

    /* CANDLE AND ANIMATIONS */
    public final CANdle m_CANdle = new CANdle(Config.kCandle, Config.kCanbus); // Ignore errors regarding the CANdle, it errantly thinks that com.ctre.phoenix can't be resolved.

    /* OTHER VARIABLES */
    private float f_Brightness = 1;
    private int[] i_CurrentColor = {0,0,0};
    private boolean b_Raw = false;

    /**
     * This is the constructor for the LED subsystem.
     */
    public LEDSubsystem() {
        e_ColorTarget = Color.White;
        e_PatternTarget = Pattern.Solid;
        m_CANdle.configFactoryDefault();
    }

    @Override
    public void periodic() {
        if (i_Timer <= 0) {
            i_Timer = 90;
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                    Color.Alliance.r = 255;
                    Color.Alliance.g = 0;
                    Color.Alliance.b = 0;
                }
                else {
                    Color.Alliance.r = 0;
                    Color.Alliance.g = 0;
                    Color.Alliance.b = 255;
                }
            }
            else {
                Color.Alliance.r = 255;
                Color.Alliance.g = 255;
                Color.Alliance.b = 255;
            }
        }
        i_Timer--;

        m_CANdle.setLEDs(
            Math.round(i_CurrentColor[0]*f_Brightness),
            Math.round(i_CurrentColor[1]*f_Brightness),
            Math.round(i_CurrentColor[2]*f_Brightness)
        );       
    }

    /**
     * Get the RGB of the color target.
     * @return
     */
    private int[] getRGB() {
        int[] rgb = {e_ColorTarget.r, e_ColorTarget.g, e_ColorTarget.b};
        return rgb;
    }

    /**
     * Sets the color of the LED subsystem using a Color enum value
     * @param color A Color enum value
     */
    public void setColor(Color color) {
        e_ColorTarget = color;
        f_Brightness = 1;
        b_Raw = false;
        i_CurrentColor = getRGB();
    }

    /**
     * Sets the color of the LED subsystem using raw RGB values
     * @param r The red channel
     * @param g The green channel
     * @param b The blue channel
     */
    public void setColorRaw(int r, int g, int b) {
        int[] array = {r,g,b};
        i_CurrentColor = array;
        b_Raw = true;
    }

    /**
     * Sets the pattern of the LED subsystem using a Pattern enum value
     * @param pattern A Pattern enum value
     */
    public void setPattern(Pattern pattern) {
        e_PatternTarget = pattern;
    }

    /**
     * Sets the brightness of the LED subsystem using a float of the range 0-1
     * @param brightness A float between 0-1
     */
    public void setBrightness(float brightness) {
        f_Brightness = brightness;
    }
}