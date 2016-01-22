package org.robocracy.ftcrobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.robocracy.ftcrobot.DriveSystem.AWDMecanumDS;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.util.FileRW;
import org.robocracy.ftcrobot.util.PIDController;

import java.util.concurrent.TimeUnit;

/**
 * @author Team Robocracy
 *
 * Runs robot during Autonomous mode.
 */
public class AutonomousScorer {
    FTCRobot robot;
    LinearOpMode curOpMode;
    public ColorSensor colorSensor;
    public OpticalDistanceSensor ods;
    boolean  allianceIsBlue;

    public AutonomousScorer(FTCRobot robot, LinearOpMode curOpMode, boolean allianceIsBlue) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.allianceIsBlue = allianceIsBlue;
        try {
            this.colorSensor = curOpMode.hardwareMap.colorSensor.get("color_sensor1");
            this.ods = curOpMode.hardwareMap.opticalDistanceSensor.get("ods_sensor1");
        }
        catch(Exception e){
            DbgLog.error(String.format("%s . Device skipped", e.getMessage()));
        }
    }

    public void step1_driveToRepairZone(AWDMecanumDS drivesys) throws InterruptedException{
        int[] colorIDs = new int[2];
        double rli;
        int distance; // inches
        int angle;

        // First, bring down the arm holding the sensors.
        // The arm was in folded position to fit the robot into 18" cube.
        //colorServo.setDirection(Servo.Direction.FORWARD);
        //colorServo.setPosition(0.5); // Need to test multiple values to find the correct one.

        // Now move towards the rescue beacon repair zone.
        if (this.allianceIsBlue) {
            //colorIDs[0] = 1;
            //colorIDs[1] = 5;  // To be modified with the IDs corresponding to White and blue
            //rli = 0.05;
            //distance = 77; // inches
            //angle = 15;
            //double omega = 50;
            drivesys.autoMecanum(90, 12, 12, -40);
            //drivesys.autoMecanum(0, , 12, -40);
            drivesys.autoMecanum(90, 72, 12, 0);

        }
        else
        {
            colorIDs[0] = 1;
            colorIDs[1] = 5;  // To be modified with the IDs corresponding to White and red
            //rli = 0.05;
            //distance = 77; // inches
            //angle = 165;
            drivesys.autoMecanum(90, 12, 12, 40);
            drivesys.autoMecanum(90, 72, 12, 0);
        }

        double speed = 12; // inches per second
        //drivesys.autoMecanumUntil(angle, speed, distance,colorIDs, rli, this);
        //drivesys.autoMecanum(angle, distance, speed, );
        //drivesys.autoMecanum(270, 4, 6, 0);
    }

    public void step2_alignWithWhiteLine(AWDMecanumDS drivesys) throws InterruptedException {
        int[] colorIDs = new int[2];
        double rli;
        int distance; // inches
        double speed;
        int angle;

        // If we are in blue alliance, strafe to the left until the white line is seen;
        // Otherwise strafe to the right.
        if (this.allianceIsBlue) {
            angle = 180;
        }
        else { // red alliance
            angle = 0;
        }
        // The robot should be able to find the white line within 24 inches from where it stopped in step 1
        distance = 24;
        rli = 0.2;
        colorIDs[0] = 1; // for white color; need to verify
        speed = 12;
        drivesys.autoMecanumUntil(angle, speed, distance,colorIDs, rli, this);
    }

    public void step3_moveToTheRescueBeacon(AWDMecanumDS drivesys) throws InterruptedException {
        // Follow the white line slowly until the touch sensor is pressed.
        PIDController lfPID = new PIDController(0.1, 0, 0.1, 4, 0.2, 0.3);

    }

    public void step4_moveBackToMountainBase(AWDMecanumDS drivesys) throws InterruptedException {
        // If in blue alliance, move back in ~ 315 angle for ~ 36 inches
        // Else, move back in ~ 225 angle for ~ 36 inches

        // If in blue alliance, spin right 90 degrees
        // Else, spin left 90 degrees

        // Fold the sensor arm again, as it will come in the way climbing the mountian.
        //colorServo.setPosition(0); // Need to test multiple values to find the correct one.

        // Move forward ~ 36 inches
        drivesys.autoMecanum(90, 36, 12, 0);
    }

    /**
     * Drives robot during Autonomous based on values recorded in .csv file at {@code filepath}.
     * @throws InterruptedException
     */
    public void driveUsingReplay() throws InterruptedException {
        DriverCommand drvrCmd;
        long replayStartTime;
        FileRW readFileRW;
        readFileRW = this.robot.readFileRW;

//        String filePath = "/sdcard/FIRST/autonomousLog/" + System.nanoTime() + ".csv";
//        robot.setFileHandle(filePath, true);
//        FileRW fileRW = new FileRW(filepath, false);

        String line = readFileRW.getNextLine();
        // Note the starting timestamp
        replayStartTime = System.nanoTime();
        while (line != null){
            this.curOpMode.waitForNextHardwareCycle();
            drvrCmd = robot.drvrStation.getNextCommand(line);
            DbgLog.msg(String.format("line = %s", line));
            if ((System.nanoTime() - replayStartTime) < drvrCmd.timeStamp) {
                // Wait for a few nano seconds
                // This will not be precise but that should be okay
                TimeUnit.NANOSECONDS.sleep(drvrCmd.timeStamp - ((System.nanoTime() - replayStartTime)));
            }
            robot.driveSys.applyCmd(drvrCmd);
            robot.linearLift.applyCmd(drvrCmd);
            line = readFileRW.getNextLine();
        }
        robot.driveSys.stopDriveSystem();
        this.curOpMode.waitForNextHardwareCycle();
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