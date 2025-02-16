// This command is a wrapper for some orchestra functions.
// It relieves the other subsystems of their duties to safely play music.
// It also monitors the power supplied to music devices to activate LEDs

package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class PlaySong extends Command {
    private final CommandSwerveDrivetrain s_Swerve;

    private final Orchestra o_Orchestra;
    private final String str_song;
    private final GenericHID xb_Operator;
    private int i_Delay;

    private final int ctrl_SongCancel = XboxController.Button.kStart.value;

    /**
     * This is the constructor for the PlaySong command. 
     * Be aware that scheduling this command will enter the robot into a vegetative state as it enjoys the music too.
     * @param orchestra The Orchestra object
     * @param swerve The Swerve Subsystem
     * @param intake The Intake Subsystem
     * @param shooter The Shooter Subsystem
     * @param climber The Climber Subsystem
     * @param song The song to play, provided as a string path
     * @param xbox The xbox controller that can disable the song
     * @param led The LED subsystem
     */
    public PlaySong(Orchestra orchestra, CommandSwerveDrivetrain swerve, String song, GenericHID xbox) {
        o_Orchestra = orchestra;
        s_Swerve = swerve;
        str_song = song;
        xb_Operator = xbox;
        addRequirements(s_Swerve);//,s_Climber);
    }

    /**
     * This runs when the PlaySong command initializes.
     * It prints the current song to the console and loads the provided song into the orchestra.
     * Then, it plays and creates a delay before the song can be disabled.
     * Also, the LEDS are set to a rainbow pattern.
     */
    @Override
    public void initialize() {
        System.out.println("Playing "+str_song+"!");
        o_Orchestra.loadMusic(str_song);
        o_Orchestra.play();
        i_Delay = 50;
    }

    /**
     * This function runs repeatedly while PlaySong is scheduled.
     * It converts the song output into LED brightness (in theory, still needs testing).
     * It also awaits controller input to be disabled.
     */
    @Override
    public void execute() {
        i_Delay --;
        // double output = pollOrchOutput();
        // if (output > f_RealOutput) {
        //     f_RealOutput = (float) output;
        // }
        // s_LED.setBrightness(f_RealOutput);
        // f_RealOutput *= 0.99;
        if (i_Delay < 0 && xb_Operator.getRawButton(ctrl_SongCancel)) {
            o_Orchestra.stop();
            System.out.println("Orchestra finished!");
        }
        // SmartDashboard.putNumber("Orchestra - Output", f_RealOutput);
    }

    @Override
    public boolean isFinished() {
        return !o_Orchestra.isPlaying();
    }
}