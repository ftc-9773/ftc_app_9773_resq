package org.robocracy.ftcrobot.DriverStation;

import org.robocracy.ftcrobot.FTCRobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * @author Team Robocracy
 *
 * Contains methods that create {@link DriverCommand}s based on various inputs.
 */
public class DriverStation {
    public static DriverCommand drvrCmd = new DriverCommand();

    FTCRobot robot;
    LinearOpMode curOpMode;

    public DriverStation(LinearOpMode curOpMode, FTCRobot robot) {
        this.curOpMode = curOpMode;
        this.robot = robot;
    }

    /**
     * Gets x and y coordinates from gamepad1 (driving gamepad) and calculates values to write to {@link DriverCommand#drvsyscmd} object.
     */
    private void getNextDrivesysCmd() {
        int moveAngle = 0;
        double x = curOpMode.gamepad1.left_stick_x;
        double y = -curOpMode.gamepad1.left_stick_y;

        //Calculate what angle robot must move in based on 8 zones of joystick
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

        //Write values to drvrCmd
        drvrCmd.drvsyscmd.angle = moveAngle;
        drvrCmd.drvsyscmd.speedMultiplier = speed;
        drvrCmd.drvsyscmd.Omega = -curOpMode.gamepad1.right_stick_x;

    }

    /*private void getNextHarvesterCmd(){
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

    }*/

    /**
     * Gets Y values of gamepad 2 (attachment gamepad) and writes values into {@link DriverCommand#linliftcmd} object.
     */
    private void getNextLinearLiftCmd(){
        float angle = -curOpMode.gamepad2.left_stick_y;
        float direction = -curOpMode.gamepad2.right_stick_y;

        drvrCmd.linliftcmd.direction = Range.clip(direction, -1,1);
        drvrCmd.linliftcmd.angle = Range.clip(angle, -1, 1);
    }

    /**
     * Gets button values of gamepad 1 (driving gamepad) and writes {@code enum} into {@link DriverCommand#latchCmd} object.
     */
    private void getNextLatchCmd(){
        boolean latchDown = curOpMode.gamepad1.a;
        boolean latchUp = curOpMode.gamepad1.y;

        if(latchDown == true){
            drvrCmd.latchCmd.direction = DriverCommand.LatchDirection.DOWN;
        }
        else if(latchUp == true){
            drvrCmd.latchCmd.direction = DriverCommand.LatchDirection.UP;
        }
        else{
            drvrCmd.latchCmd.direction = DriverCommand.LatchDirection.NONE;
        }
    }

    /**
     * Calls {@link DriverStation#getNextDrivesysCmd()}, {@link DriverStation#getNextLinearLiftCmd()}, and {@link DriverStation#getNextLatchCmd()}.
     *
     * @return {@link DriverCommand} object with all values
     */
    public DriverCommand getNextCommand() {

        getNextDrivesysCmd();
        //getNextHarvesterCmd();
        getNextLinearLiftCmd();
        //getNextLatchCmd();

        return (drvrCmd);
    }

    /**
     * Overrides {@link DriverStation#getNextCommand()}.
     *
     * @see org.robocracy.ftcrobot.AutonomousScorer#driveUsingReplay(String filePath)
     * @param line Line of comma-seperated values in csv file read in {@link org.robocracy.ftcrobot.AutonomousScorer#driveUsingReplay(String filePath)}
     * @return {@link DriverCommand#drvsyscmd} object with values.
     */
    public DriverCommand getNextCommand(String line){
        String[] lineArray = line.split(",");

        double angle = Double.parseDouble(lineArray[1]);
        double speedMultiplier = Double.parseDouble(lineArray[2]);
        double Omega = Double.parseDouble(lineArray[3]);

        drvrCmd.drvsyscmd.angle = angle;
        drvrCmd.drvsyscmd.Omega = Omega;
        drvrCmd.drvsyscmd.speedMultiplier = speedMultiplier;
        return (drvrCmd);
    }


}
