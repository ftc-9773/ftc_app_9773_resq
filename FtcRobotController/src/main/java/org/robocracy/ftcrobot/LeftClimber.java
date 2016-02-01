package org.robocracy.ftcrobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.Servo;

import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.FTCRobot;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Operates Climber release servos on robot
 * @author Team Robocracy
 */
public class LeftClimber {
    FTCRobot robot;
    LinearOpMode curOpMode;
    Servo leftClimber;
    boolean leftClimberAvailable = false;

    public LeftClimber(FTCRobot robot, Servo leftClimber, LinearOpMode curOpMode, boolean allianceIsBlue){
        this.robot = robot;
        this.curOpMode = curOpMode;
        if (leftClimber != null) {
            leftClimberAvailable = true;
            this.leftClimber = leftClimber;
            this.leftClimber.setDirection(Servo.Direction.REVERSE);
            DbgLog.msg(String.format("Left climber position = %f", this.leftClimber.getPosition()));
            this.leftClimber.setPosition(1.0);
            if(!allianceIsBlue){
                this.leftClimber.close();
            }
        }
    }

    /**
     * Moves latch servos based on {@code enum direction} value set in {@link DriverStation#getNextLatchCmd()}
     * @param drvrcmd {@link DriverCommand} object with values.
     */
    public void applyDSCmd(DriverCommand drvrcmd){
        if (!leftClimberAvailable) {
            return;
        }
        double leftClimberPosition = leftClimber.getPosition();
        DbgLog.msg(String.format("Left climber position = %f", leftClimberPosition));
        switch (drvrcmd.leftClimberCmd.leftClimberStatus){
            case -1:
                leftClimber.setPosition(Range.clip(leftClimberPosition+0.01, 0, 1));
                break;
            case 1:
                leftClimber.setPosition(Range.clip(leftClimberPosition-0.01, 0, 1));
                break;
            case 0:
                break;
            default:
                break;
        }
    }
}
