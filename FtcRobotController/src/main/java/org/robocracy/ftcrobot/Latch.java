package org.robocracy.ftcrobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.DriverStation.DriverStation;

/**
 * Operates latches on robot that hold on to churros
 * @author Team Robocracy
 */
public class Latch {
    FTCRobot robot;
    LinearOpMode curOpMode;
    Servo leftLatch;
    Servo rightLatch;
    boolean latchAvailable = false;

    public Latch(FTCRobot robot, Servo leftLatch, Servo rightLatch, LinearOpMode curOpMode){
        this.robot = robot;
        this.curOpMode = curOpMode;
        if (leftLatch != null && rightLatch != null) {
            latchAvailable = true;
            this.leftLatch = leftLatch;
            this.leftLatch.scaleRange(0, 0.35294);
            this.leftLatch.setPosition(0); // 1.0 is actually 0.35294 because of the above scaleRange call
            this.rightLatch = rightLatch;
            this.rightLatch.scaleRange(0.4705, 1);
            this.rightLatch.setDirection(Servo.Direction.REVERSE);
            this.rightLatch.setPosition(0); // 1.0 == 0.4705 because of scaleRange call  & REVERSE setting above
            double leftPosition = leftLatch.getPosition();
            double rightPosition = rightLatch.getPosition();
            DbgLog.msg(String.format("LeftLatchPosition = %f, RightLatchPosition = %f", leftPosition, rightPosition));
        }
    }

    /**
     * Moves latch servos based on {@code enum direction} value set in {@link DriverStation#getNextLatchCmd()}
     * @param drvrcmd {@link DriverCommand} object with values.
     */
    public void applyDSCmd(DriverCommand drvrcmd){
        if (!latchAvailable) {
            return;
        }
        double leftPosition = leftLatch.getPosition();
        double rightPosition = rightLatch.getPosition();
//        DbgLog.msg(String.format("LeftLathPosition = %f, RightLatchPosition = %f", leftPosition, rightPosition));
        switch (drvrcmd.latchCmd.latchStatus){
            case -2:
                leftLatch.setPosition(Range.clip(0.2, 0, 1));
                rightLatch.setPosition(Range.clip(0.2, 0, 1));
                break;
            case -1:
                leftLatch.setPosition(Range.clip(0, 0, 1));
                rightLatch.setPosition(Range.clip(0, 0, 1));
                break;
            case 1:
                leftLatch.setPosition(Range.clip(1, 0, 1));
                rightLatch.setPosition(Range.clip(1, 0, 1));
                break;
            case 0:
                break;
            default:
                break;
        }
    }
}
