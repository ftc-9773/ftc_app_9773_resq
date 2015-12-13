package org.robocracy.ftcrobot;

import com.qualcomm.robotcore.hardware.Servo;
import org.robocracy.ftcrobot.FTCRobot;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by pranavb on 12/5/15.
 */
public class Latch {
    FTCRobot robot;
    LinearOpMode curOpMode;
    Servo leftLatch;
    Servo rightLatch;

    public Latch(FTCRobot robot, Servo leftLatch, Servo rightLatch, LinearOpMode curOpMode){
        this.curOpMode = curOpMode;
        this.leftLatch = leftLatch;
        this.rightLatch = rightLatch;
        this.robot = robot;
    }

    public void applyDSCmd(DriverCommand drvrcmd){
        double leftPosition = leftLatch.getPosition();
        double rightPosition = rightLatch.getPosition();
        switch (drvrcmd.latchCmd.direction){
            case DOWN:
                leftLatch.setPosition(Range.clip((leftPosition + 0.05), 0, 1));
                rightLatch.setPosition(Range.clip((rightPosition + 0.05), 0, 1));
                break;
            case UP:
                leftLatch.setPosition(Range.clip((leftPosition - 0.05), 0, 1));
                rightLatch.setPosition(Range.clip((rightPosition - 0.05), 0, 1));
                break;
            case NONE:
                break;
            default:
                break;
        }
    }
}
