package org.robocracy.ftcrobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.robocracy.ftcrobot.DriveSystem.AWDMecanumDS;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.util.FileRW;

import java.util.concurrent.TimeUnit;

/**
 * Runs robot during Autonomous mode.
 * @author Team Robocracy
 */
public class AutonomousScorer {
    FTCRobot robot;
    LinearOpMode curOpMode;
    boolean  allianceIsBlue;

    public AutonomousScorer(FTCRobot robot, LinearOpMode curOpMode, boolean allianceIsBlue) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.allianceIsBlue = allianceIsBlue;
    }

    /**
     * Drives robot during Autonomous based on values recorded in .csv file at {@code filepath}.
     * @throws InterruptedException
     */
    public void driveUsingReplay() throws InterruptedException {
        DriverCommand drvrCmd, tmpDrvrCmd;
        long replayStartTime;
        FileRW readFileRW;
        readFileRW = this.robot.readFileRW;

        String line = readFileRW.getNextLine();
        // Note the starting timestamp
        replayStartTime = System.nanoTime();
        while (line != null){
            this.curOpMode.waitForNextHardwareCycle();
            drvrCmd = robot.drvrStation.getNextCommand(line);
            if ((System.nanoTime() - replayStartTime) < drvrCmd.timeStamp) {
                // Wait for a few nano seconds
                // This will not be precise but that should be okay
                TimeUnit.NANOSECONDS.sleep(drvrCmd.timeStamp - ((System.nanoTime() - replayStartTime)));
            }
            robot.driveSys.applyCmd(drvrCmd);
//            robot.linearLift.applyCmd(drvrCmd);
            drvrCmd.harvestercmd.direction = DriverCommand.HarvesterDirection.PUSH;
            robot.harvester.applyDSCmd(drvrCmd);
            drvrCmd.latchCmd.latchStatus = -2;
            robot.latch.applyDSCmd(drvrCmd);
            robot.climberDispenser.applyDSCmd(drvrCmd);
            line = readFileRW.getNextLine();
        }
        robot.driveSys.stopDriveSystem();
        tmpDrvrCmd = new DriverCommand();
        tmpDrvrCmd.harvestercmd.direction = DriverCommand.HarvesterDirection.NONE;
        robot.harvester.applyDSCmd(tmpDrvrCmd);
        this.curOpMode.waitForNextHardwareCycle();
    }

    public boolean findTheWhiteLine() throws InterruptedException {
        boolean whiteLineFound = false;
        boolean spun45degrees = false;
        boolean spun90degrees = false;
        DriverCommand tmpDrvrCmd = new DriverCommand();
        float initialYaw = robot.navxDevice.getYaw();

        if (robot.ods.getLightDetected() > 0.1) {
            whiteLineFound = true;
        }
        while (!whiteLineFound && !spun45degrees){

            DbgLog.error(String.format("First Loop: rotation = %f", robot.navxDevice.getYaw() - initialYaw));
            tmpDrvrCmd.drvsyscmd.angle = 0;
            tmpDrvrCmd.drvsyscmd.speedMultiplier = 0;
            tmpDrvrCmd.drvsyscmd.Omega = 0.5;
            robot.driveSys.applyCmd(tmpDrvrCmd);
            if (robot.ods.getLightDetected() > 0.1) {
                whiteLineFound = true;
            }
            if (Math.abs(robot.navxDevice.getYaw() - initialYaw) >= 45) {
                spun45degrees = true;
            }
            this.curOpMode.waitForNextHardwareCycle();
        }
        // Now spin in the opposite direction
        initialYaw = robot.navxDevice.getYaw();
        while (!whiteLineFound && !spun90degrees){
            DbgLog.error(String.format("Second Loop: rotation = %f", robot.navxDevice.getYaw() - initialYaw));

            tmpDrvrCmd.drvsyscmd.angle = 0;
            tmpDrvrCmd.drvsyscmd.speedMultiplier = 0;
            tmpDrvrCmd.drvsyscmd.Omega = -0.5;
            robot.driveSys.applyCmd(tmpDrvrCmd);
            if (robot.ods.getLightDetected() > 0.1) {
                whiteLineFound = true;
            }
            if (Math.abs(robot.navxDevice.getYaw() - initialYaw) >= 90) {
                spun90degrees = true;
            }
            this.curOpMode.waitForNextHardwareCycle();
        }
        // Now stop the robot
        tmpDrvrCmd.drvsyscmd.angle = 0;
        tmpDrvrCmd.drvsyscmd.speedMultiplier = 0;
        tmpDrvrCmd.drvsyscmd.Omega = 0;
        robot.driveSys.applyCmd(tmpDrvrCmd);
        this.curOpMode.waitForNextHardwareCycle();

        return (whiteLineFound);
    }

    public void rotateToAngle(float targetYaw) throws InterruptedException {
        DriverCommand tmpDrvrCmd = new DriverCommand();
        float initialYaw = robot.navxDevice.getYaw();
        float angleToSpin = Math.abs(initialYaw - targetYaw);
        int directionIndicator;
        boolean reachedTargetYaw = false;

        DbgLog.msg(String.format("TargetYaw=%f, initialYaw=%f", targetYaw, initialYaw));

        directionIndicator = (targetYaw > robot.navxDevice.getYaw() ? -1 : 1);
        while (!reachedTargetYaw) {
            tmpDrvrCmd.drvsyscmd.angle = 0;
            tmpDrvrCmd.drvsyscmd.speedMultiplier = 0;
            tmpDrvrCmd.drvsyscmd.Omega = directionIndicator * 0.5;
            robot.driveSys.applyCmd(tmpDrvrCmd);
            if (Math.abs(robot.navxDevice.getYaw() - initialYaw) >= angleToSpin) {
                reachedTargetYaw = true;
            }
            this.curOpMode.waitForNextHardwareCycle();
        }
        // Now stop the robot
        tmpDrvrCmd.drvsyscmd.angle = 0;
        tmpDrvrCmd.drvsyscmd.speedMultiplier = 0;
        tmpDrvrCmd.drvsyscmd.Omega = 0;
        robot.driveSys.applyCmd(tmpDrvrCmd);
        this.curOpMode.waitForNextHardwareCycle();
    }

    /**
     * Drives robot during End Game based on values recorded in .csv file
     * @param isEndGame parameter that, when provided, overrides {@link #driveUsingReplay()}.
     * @throws InterruptedException
     */
    public void driveUsingReplay(boolean isEndGame) throws InterruptedException{
        if(isEndGame) {
            DriverCommand drvrCmd;
            long replayStartTime;
            FileRW readFileRW;
            readFileRW = this.robot.readFileRW;

            String line = readFileRW.getNextLine();
            // Note the starting timestamp
            replayStartTime = System.nanoTime();
            while (line != null) {
                this.curOpMode.waitForNextHardwareCycle();
                drvrCmd = robot.drvrStation.getNextCommand(line, true);
//                DbgLog.msg(String.format("line = %s", line));
                if ((System.nanoTime() - replayStartTime) < drvrCmd.timeStamp) {
                    // Wait for a few nano seconds
                    // This will not be precise but that should be okay
                    TimeUnit.NANOSECONDS.sleep(drvrCmd.timeStamp - ((System.nanoTime() - replayStartTime)));
                }
                robot.driveSys.applyCmd(drvrCmd);
                robot.linearLift.applyCmd(drvrCmd);
                robot.latch.applyDSCmd(drvrCmd);
                robot.rightClimber.applyDSCmd(drvrCmd);
                robot.leftClimber.applyDSCmd(drvrCmd);
                line = readFileRW.getNextLine();
            }
            robot.driveSys.stopDriveSystem();
            this.curOpMode.waitForNextHardwareCycle();
        }
    }

    public void strafeTheDistance(AWDMecanumDS drivesys, int distanceToStrafe) throws InterruptedException {
        int angle, speed;
        if (distanceToStrafe < 0) {
            angle = 180;
        }
        else {
            angle = 0;
        }
        speed = 12;
        drivesys.autoMecanum(angle, distanceToStrafe, speed, 0);
    }
}