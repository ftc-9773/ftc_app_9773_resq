package org.robocracy.ftcrobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.Servo;

import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.FTCRobot;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * @author Team Robocracy
 *
 * Operates latches on robot that hold on to churros
 */
public class LeftClimber {
    FTCRobot robot;
    LinearOpMode curOpMode;
    Servo leftClimber;

    public LeftClimber(FTCRobot robot, Servo leftClimber, LinearOpMode curOpMode){
        this.curOpMode = curOpMode;
        this.leftClimber = leftClimber;
        this.robot = robot;
    }

    /**
     * Moves latch servos based on {@code enum direction} value set in {@link DriverStation#getNextLatchCmd()}
     * @param drvrcmd {@link DriverCommand} object with values.
     */
    public void applyDSCmd(DriverCommand drvrcmd){
        DbgLog.msg(String.format("Left climber position = %f", leftClimber.getPosition()));
        switch (drvrcmd.leftClimberCmd.leftClimberDirection){
            case DOWN:
                leftClimber.setPosition(1);
                break;
            case UP:
                leftClimber.setPosition(0);
                break;
            case NONE:
                break;
            default:
                break;
        }
    }
}
