package org.robocracy.ftcrobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.robocracy.ftcrobot.DriverStation.DriverCommand;

/**
 * Created by pranavb on 2/20/16.
 */
public class SignalReleaser {
    FTCRobot robot;
    LinearOpMode curOpMode;
    Servo signalReleaseServo;
    boolean signalReleaserAvailable = false;

    public SignalReleaser(FTCRobot robot, Servo signalReleaseServo, LinearOpMode curOpMode){
        this.robot = robot;
        this.curOpMode = curOpMode;
        if(signalReleaseServo != null){
            signalReleaserAvailable = true;
            this.signalReleaseServo = signalReleaseServo;
            this.signalReleaseServo.scaleRange(0.06, 0.67);
            this.signalReleaseServo.setDirection(Servo.Direction.REVERSE);
            this.signalReleaseServo.setPosition(0);

        }
    }

    public void applyDSCmd(DriverCommand driverCommand){
        if(!signalReleaserAvailable){
            return;
        }

        switch(driverCommand.signalReleaseCommand.signalReleaseStatus){
            case UP:
                signalReleaseServo.setPosition(0);
                break;
            case DOWN:
                signalReleaseServo.setPosition(1);
                break;
            default:
                break;
        }
    }
}
