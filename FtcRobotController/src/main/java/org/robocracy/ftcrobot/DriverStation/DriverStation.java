package org.robocracy.ftcrobot.DriverStation;

import org.robocracy.ftcrobot.FTCRobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Robocracy on 11/19/2015.
 */
public class DriverStation {
    public static DriverCommand drvrCmd = new DriverCommand();

    FTCRobot robot;
    LinearOpMode curOpMode;

    public DriverStation(LinearOpMode curOpMode, FTCRobot robot) {
        this.curOpMode = curOpMode;
        this.robot = robot;
    }

    private void getNextDrivesysCmd() {
        int moveAngle = 0;
        double x = curOpMode.gamepad1.left_stick_x;
        double y = -curOpMode.gamepad1.left_stick_y;

        if (x == 0 && y == 0) {
            moveAngle = 0;
        } else if (x == 0) {
            if (y > 0) {
                moveAngle = 90;
            } else {
                moveAngle = 270;
            }
        } else if (y == 0) {
            if (x > 0) {
                moveAngle = 0;
            } else {
                moveAngle = 180;
            }
        } else {
            int moveAngleRaw = (int) (Math.toDegrees(Math.atan(Math.abs(y / x))));

            if ((x < 0) && (y > 0)) {
                moveAngleRaw += 90;
            } else if ((x < 0) && (y < 0)) {
                moveAngleRaw += 180;
            } else if ((x > 0) && (y < 0)) {
                moveAngleRaw += 270;
            }

            moveAngleRaw += 22.5;
            moveAngleRaw %= 360;
            int sector = moveAngleRaw / 45;
            moveAngle = sector * 45;
        }
        double speed = Math.sqrt((x * x) + (y * y)) / Math.sqrt(2);

//        DbgLog.msg(String.format("x = %f, y= %f, moveAngle = %d",
//                x, y, moveAngle));
        drvrCmd.drvsyscmd.angle = moveAngle;
        drvrCmd.drvsyscmd.speedMultiplier = speed;
        drvrCmd.drvsyscmd.Omega = -curOpMode.gamepad1.right_stick_x;

    }

    private void getNextHarvesterCmd(){
        boolean harvesterPull = curOpMode.gamepad2.right_bumper;
        float harvesterPushSlow = curOpMode.gamepad2.right_trigger;
        boolean harvesterPush = false;

        if(harvesterPushSlow > 0.3){
            drvrCmd.harvestercmd.direction = DriverCommand.HarvesterDirection.PUSH;
        }
        else if(harvesterPull == true){
            drvrCmd.harvestercmd.direction = DriverCommand.HarvesterDirection.PULL;
        }
        else {
            drvrCmd.harvestercmd.direction = DriverCommand.HarvesterDirection.NONE;
        }

    }

    private void getNextLinearLiftCmd(){
        float angle = -curOpMode.gamepad2.left_stick_y;
        float direction = -curOpMode.gamepad2.right_stick_y;

        drvrCmd.linliftcmd.direction = Range.clip(direction, -1,1);
        drvrCmd.linliftcmd.angle = Range.clip(angle, -1, 1);
    }

    public DriverCommand getNextCommand() {

        getNextDrivesysCmd();
        getNextHarvesterCmd();
        getNextLinearLiftCmd();

        return (drvrCmd);
    }


}
