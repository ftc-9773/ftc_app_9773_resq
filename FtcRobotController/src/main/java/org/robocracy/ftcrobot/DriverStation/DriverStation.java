package org.robocracy.ftcrobot.DriverStation;

import org.robocracy.ftcrobot.FTCRobot;
import org.robocracy.ftcrobot.util.NavX;

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

    /**
     * Gets right bumper and trigger values of gamepad 2 (attachment gamepad) and writes values into {@link DriverCommand#harvestercmd} object.
     */
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

    /**
     * Gets Y values of gamepad 2 (attachment gamepad) and writes values into {@link DriverCommand#linliftcmd} object.
     */
    private void getNextLinearLiftCmd(){
        float angle = -curOpMode.gamepad2.left_stick_y;
        float armLength = -curOpMode.gamepad2.right_stick_y;

        drvrCmd.linliftcmd.armLength = Range.clip(armLength, -1,1);
        drvrCmd.linliftcmd.angle = Range.clip(angle, -1, 1);
    }

    /**
     * Gets dpad values for gamepad 2 (attachment gamepad) and writes {@code enum DriverCommand.BucketCommand.BucketDirection} into {@link DriverCommand#bucketCmd} object.
     */
    private void getNextBucketCmd(){
        boolean bucketRight = curOpMode.gamepad2.dpad_right;
        boolean bucketLeft = curOpMode.gamepad2.dpad_left;

        if (bucketRight){
            drvrCmd.bucketCmd.direction = DriverCommand.BucketDirection.RIGHT;
        }
        else if (bucketLeft){
            drvrCmd.bucketCmd.direction = DriverCommand.BucketDirection.LEFT;
        }
        else{
            drvrCmd.bucketCmd.direction = DriverCommand.BucketDirection.NONE;
        }
    }
    /**
     * Gets button values of gamepad 1 (driving gamepad) and writes {@code enum DriverCommand.LatchDirection} into {@link DriverCommand#latchCmd} object.
     */
    private void getNextLatchCmd(){
        boolean latchDown = curOpMode.gamepad1.a;
        boolean latchUp = curOpMode.gamepad1.y;

        if(latchDown == true){
            drvrCmd.latchCmd.latchStatus = -1;
        }
        else if(latchUp == true){
            drvrCmd.latchCmd.latchStatus = 1;
        }
        else{
            drvrCmd.latchCmd.latchStatus = 0;
        }
    }

    /**
     * Gets button values of gamepad 2 (attachment gamepad) and writes {@code enum DriverCommand.LeftClimberDirection} and {@code enum DriverCommand.RightClimberDirection} into {@link DriverCommand#leftClimberCmd} and {@link DriverCommand#rightClimberCmd} objects respectively.
     */
    private void getNextClimberCmd(){
        boolean rightClimberDown = curOpMode.gamepad2.a;
        boolean rightClimberUp = curOpMode.gamepad2.y;
        boolean leftClimberDown = curOpMode.gamepad2.x;
        boolean leftClimberUp = curOpMode.gamepad2.b;

        if(rightClimberDown){
            drvrCmd.rightClimberCmd.rightClimberStatus = -1;
        }
        else if(rightClimberUp){
            drvrCmd.rightClimberCmd.rightClimberStatus = 1;
        }
        else{
            drvrCmd.rightClimberCmd.rightClimberStatus = 0;
        }

        if (leftClimberDown){
            drvrCmd.leftClimberCmd.leftClimberStatus = -1;
        }
        else if(leftClimberUp){
            drvrCmd.leftClimberCmd.leftClimberStatus = 1;
        }
        else {
            drvrCmd.leftClimberCmd.leftClimberStatus = 0;
        }
    }

    /**
     * Gets dPad values of gamepad 1, and writes {@code int climberDispenserStatus} into {@link org.robocracy.ftcrobot.DriverStation.DriverCommand#climberDispenserCommand}
     */
    private void getNextClimberDispenserCmd(){
        boolean climberDispenserDown = curOpMode.gamepad1.dpad_down;
        boolean climberDispenserUp = curOpMode.gamepad1.dpad_up;

        if(climberDispenserUp){
            drvrCmd.climberDispenserCommand.climberDispenserStatus = 1;
        }
        else if(climberDispenserDown){
            drvrCmd.climberDispenserCommand.climberDispenserStatus = -1;
        }
        else{
            drvrCmd.climberDispenserCommand.climberDispenserStatus = 0;
        }
        DbgLog.msg(String.format("climberDispenserStatus = %d", drvrCmd.climberDispenserCommand.climberDispenserStatus));
    }

    /**
     * Gets button value of gamepad 1 bumper, and, if pressed, starts end game.
     */
    private void getNextEndGameCmd(){
        boolean startEndGame = curOpMode.gamepad1.right_bumper;

        if(startEndGame){
            drvrCmd.runEndGame.endGameStatus = DriverCommand.EndGameStatus.RUN;
        }
        else if(!startEndGame){
            drvrCmd.runEndGame.endGameStatus = DriverCommand.EndGameStatus.STOP;
        }
        else {
            drvrCmd.runEndGame.endGameStatus = DriverCommand.EndGameStatus.NONE;
        }
    }

    /**
     * Calls {@link DriverStation#getNextDrivesysCmd()}, {@link DriverStation#getNextLinearLiftCmd()}, and {@link DriverStation#getNextLatchCmd()}.
     *
     * @return {@link DriverCommand} object with all values
     */
    public DriverCommand getNextCommand() {

        getNextDrivesysCmd();
        getNextHarvesterCmd();
        getNextLinearLiftCmd();
        getNextLatchCmd();
        getNextBucketCmd();
        getNextClimberCmd();
        getNextClimberDispenserCmd();

        if(robot.curStatus == FTCRobot.currentlyRecording.RECORDING_AUTONOMOUS){
            int angle = (int) drvrCmd.drvsyscmd.angle;
            int climberDispenserStatus = drvrCmd.climberDispenserCommand.climberDispenserStatus;
            double speedMultiplier = drvrCmd.drvsyscmd.speedMultiplier;
            double Omega = drvrCmd.drvsyscmd.Omega;
            double liftDirection = drvrCmd.linliftcmd.armLength;
            double liftAngle = drvrCmd.linliftcmd.angle;

            //Logs values into file
            if(this.robot.writeFileRW != null) {
                float[] navx_data;
                if (robot.navxDevice != null) {
                    navx_data = robot.navx_device.getNavxData();
                } else {
                    navx_data = new float[4];
                    navx_data[0] = 0;
                }
                String line = (System.nanoTime() - robot.timestamp) + "," + angle + "," + speedMultiplier + "," +
                        Omega + "," + navx_data[0] + "," + liftDirection + "," + liftAngle + "," + robot.ods.getLightDetected()
                        + "," + robot.colorSensor.red() + "," + robot.colorSensor.green() + "," + robot.colorSensor.blue() + "," + climberDispenserStatus;
                this.robot.writeFileRW.fileWrite(line);
            }
        }
        else if(robot.curStatus == FTCRobot.currentlyRecording.RECORDING_ENDGAME){
            int angle = (int) drvrCmd.drvsyscmd.angle;
            double speedMultiplier = drvrCmd.drvsyscmd.speedMultiplier;
            double Omega = drvrCmd.drvsyscmd.Omega;
            double liftDirection = drvrCmd.linliftcmd.armLength;
            double liftAngle = drvrCmd.linliftcmd.angle;
            int latchStatus = drvrCmd.latchCmd.latchStatus;
            int rightClimberStatus = drvrCmd.rightClimberCmd.rightClimberStatus;
            int leftClimberStatus = drvrCmd.leftClimberCmd.leftClimberStatus;

            //Logs values into file
            if(this.robot.writeFileRW != null) {
                float[] navx_data;
                if (robot.navxDevice != null) {
                    navx_data = robot.navx_device.getNavxData();
                } else {
                    navx_data = new float[4];
                    navx_data[0] = 0;
                }
                String line = (System.nanoTime() - robot.timestamp) + "," + angle + "," + speedMultiplier + "," +
                        Omega + "," + navx_data[0] + "," + navx_data[1] + "," + liftDirection + "," + liftAngle + "," + latchStatus + "," + rightClimberStatus + "," + leftClimberStatus;
                this.robot.writeFileRW.fileWrite(line);
            }
        }

        return (drvrCmd);
    }

    /**
     * Overrides {@link DriverStation#getNextCommand()}.
     *
     * @see org.robocracy.ftcrobot.AutonomousScorer#driveUsingReplay()
     * @param line Line of comma-seperated values in csv file read in
     * {@link org.robocracy.ftcrobot.AutonomousScorer#driveUsingReplay()}
     * @return {@link DriverCommand#drvsyscmd} object with values.
     */
    public DriverCommand getNextCommand(String line){
        long timestamp = 0;
        String[] lineArray = line.split(",");
        double angle, speedMultiplier, Omega, yaw, liftArmLengthPower, liftAnglePower, odsVal, colorRed, colorGreen, colorBlue;
        int climberDispenserStatus;
        if (lineArray.length >= 12) {
            timestamp = Long.parseLong(lineArray[0]);
            angle = Double.parseDouble(lineArray[1]);
            speedMultiplier = Double.parseDouble(lineArray[2]);
            Omega = Double.parseDouble(lineArray[3]);
            yaw = Double.parseDouble(lineArray[4]);
            liftArmLengthPower = Double.parseDouble(lineArray[5]);
            liftAnglePower = Double.parseDouble(lineArray[6]);
            odsVal = Double.parseDouble(lineArray[7]);
            colorRed = Double.parseDouble(lineArray[8]);
            colorGreen = Double.parseDouble(lineArray[9]);
            colorBlue = Double.parseDouble(lineArray[10]);
            climberDispenserStatus = Integer.parseInt(lineArray[11]);
        }
        else {
            angle = 0.0;
            speedMultiplier = 0.0;
            Omega = 0.0;
            yaw = 0.0;
            liftArmLengthPower = 0.0;
            liftAnglePower = 0.0;
            odsVal = 0.0;
            colorRed = 0.0;
            colorGreen = 0.0;
            colorBlue = 0.0;
            climberDispenserStatus = 0;
        }

        drvrCmd.timeStamp = timestamp;
        drvrCmd.drvsyscmd.angle = angle;
        drvrCmd.drvsyscmd.Omega = Omega;
        drvrCmd.drvsyscmd.speedMultiplier = speedMultiplier;
        drvrCmd.linliftcmd.angle = (float) liftAnglePower;
        drvrCmd.linliftcmd.armLength = (float) liftArmLengthPower;
        drvrCmd.sensorValues.yaw = yaw;
        drvrCmd.sensorValues.ods = odsVal;
        drvrCmd.sensorValues.colorRed = colorRed;
        drvrCmd.sensorValues.colorGreen = colorGreen;
        drvrCmd.sensorValues.colorBlue = colorBlue;
        drvrCmd.climberDispenserCommand.climberDispenserStatus = climberDispenserStatus;
        return (drvrCmd);
    }
    /**
     * Overrides {@link DriverStation#getNextCommand()}.
     *
     * @see org.robocracy.ftcrobot.EndGamePlayer#runEndGame(DriverCommand) ()
     * @param line Line of comma-seperated values in csv file read in
     * {@link org.robocracy.ftcrobot.AutonomousScorer#driveUsingReplay()}
     * @return {@link DriverCommand#drvsyscmd} object with values.
     */
    public DriverCommand getNextCommand(String line, boolean isEndGame){
        if(isEndGame) {
            long timestamp = 0;
            String[] lineArray = line.split(",");
            double angle, speedMultiplier, Omega, yaw, pitch, liftArmLengthPower, liftAnglePower, odsVal, colorRed, colorGreen, colorBlue;
            int latchStatus = 0;
            int rightClimberStatus;
            int leftClimberStatus;
            if (lineArray.length >= 15) {
                timestamp = Long.parseLong(lineArray[0]);
                angle = Double.parseDouble(lineArray[1]);
                speedMultiplier = Double.parseDouble(lineArray[2]);
                Omega = Double.parseDouble(lineArray[3]);
                yaw = Double.parseDouble(lineArray[4]);
                pitch = Double.parseDouble(lineArray[5]);
                liftArmLengthPower = Double.parseDouble(lineArray[6]);
                liftAnglePower = Double.parseDouble(lineArray[7]);
                odsVal = Double.parseDouble(lineArray[8]);
                colorRed = Double.parseDouble(lineArray[9]);
                colorGreen = Double.parseDouble(lineArray[10]);
                colorBlue = Double.parseDouble(lineArray[11]);
                latchStatus = Integer.parseInt(lineArray[12]);
                rightClimberStatus = Integer.parseInt(lineArray[13]);
                leftClimberStatus = Integer.parseInt(lineArray[14]);
            } else {
                angle = 0.0;
                speedMultiplier = 0.0;
                Omega = 0.0;
                yaw = 0.0;
                pitch = 0.0;
                liftArmLengthPower = 0.0;
                liftAnglePower = 0.0;
                odsVal = 0.0;
                colorRed = 0.0;
                colorGreen = 0.0;
                colorBlue = 0.0;
                latchStatus = 0;
                rightClimberStatus = 0;
                leftClimberStatus = 0;
            }

            drvrCmd.timeStamp = timestamp;
            drvrCmd.drvsyscmd.angle = angle;
            drvrCmd.drvsyscmd.Omega = Omega;
            drvrCmd.drvsyscmd.speedMultiplier = speedMultiplier;
            drvrCmd.linliftcmd.angle = (float) liftAnglePower;
            drvrCmd.linliftcmd.armLength = (float) liftArmLengthPower;
            drvrCmd.sensorValues.yaw = yaw;
            drvrCmd.sensorValues.pitch = pitch;
            drvrCmd.sensorValues.ods = odsVal;
            drvrCmd.sensorValues.colorRed = colorRed;
            drvrCmd.sensorValues.colorGreen = colorGreen;
            drvrCmd.sensorValues.colorBlue = colorBlue;
            drvrCmd.latchCmd.latchStatus = latchStatus;
            drvrCmd.rightClimberCmd.rightClimberStatus = rightClimberStatus;
            drvrCmd.leftClimberCmd.leftClimberStatus = leftClimberStatus;

        }
        return (drvrCmd);
    }


}