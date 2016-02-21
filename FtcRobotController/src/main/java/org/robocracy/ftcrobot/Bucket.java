package org.robocracy.ftcrobot;


import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.FTCRobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Operates bucket.
 * @author Team Robocracy
 */
public class Bucket {
    FTCRobot robot;
    LinearOpMode curOpMode;
    Servo bucketServo;
    boolean bucketAvailable = false;
    final double right = 0;
    final double left = 1;
    final double none = 0.5;

    public Bucket(FTCRobot robot, LinearOpMode curOpMode, Servo bucketServo){
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.bucketServo = bucketServo;
        if (bucketServo != null) {
            bucketAvailable = true;
        }
        if(bucketAvailable){
            this.bucketServo.setPosition(this.none);
        }
    }


    /**
     * Applies power to bucket servo based on {@code enum direction} value set in {@link DriverStation#getNextCommand()}
     * @param drvrcmd {@link DriverCommand} object with values.
     */
    public void applyDSCmd(DriverCommand drvrcmd){
        if (!bucketAvailable) {
            return;
        }
//        DbgLog.msg(String.format("Bucket Position = %f", bucketServo.getPosition()));
        switch(drvrcmd.bucketCmd.direction){
            case RIGHT:
                bucketServo.setPosition(right);
                break;
            case LEFT:
                bucketServo.setPosition(left);
                break;
            case NONE:
                bucketServo.setPosition(none);
                break;
            default:
                bucketServo.setPosition(none);
                break;
        }
    }
}
