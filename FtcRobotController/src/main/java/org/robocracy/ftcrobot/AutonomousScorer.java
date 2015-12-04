package org.robocracy.ftcrobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.robocracy.ftcrobot.DriveSystem.AWDMecanumDS;
import org.robocracy.ftcrobot.util.PIDController;

/**
 * Created by Robocracy on 12/2/2015.
 */
public class AutonomousScorer {
    FTCRobot robot;
    LinearOpMode curOpMode;
    public Servo colorServo;
    public ColorSensor colorSensor;
    public OpticalDistanceSensor ods;
    public TouchSensor touchSensor;
    boolean  allianceIsBlue;

    public AutonomousScorer(FTCRobot robot, LinearOpMode curOpMode, boolean allianceIsBlue) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.allianceIsBlue = allianceIsBlue;
        this.colorSensor = curOpMode.hardwareMap.colorSensor.get("color_sensor1");
        this.colorServo = curOpMode.hardwareMap.servo.get("colorServo");
        this.ods = curOpMode.hardwareMap.opticalDistanceSensor.get("ods_sensor1");
        this.touchSensor = curOpMode.hardwareMap.touchSensor.get("touch_sensor");
    }

    public void step1_driveToRepairZone(AWDMecanumDS drivesys) throws InterruptedException{
        int[] colorIDs = new int[2];
        double rli;
        int distance; // inches
        int angle;

        // First, bring down the arm holding the sensors.
        // The arm was in folded position to fit the robot into 18" cube.
        colorServo.setDirection(Servo.Direction.FORWARD);
        colorServo.setPosition(0.5); // Need to test multiple values to find the correct one.

        // Now move towards the rescue beacon repair zone.
        if (this.allianceIsBlue) {
            colorIDs[0] = 1;
            colorIDs[1] = 5;  // To be modified with the IDs corresponding to White and blue
            rli = 0.05;
            distance = 53; // inches
            angle = 230;
        }
        else
        {
            colorIDs[0] = 1;
            colorIDs[1] = 5;  // To be modified with the IDs corresponding to White and red
            rli = 0.05;
            distance = 53; // inches
            angle = 230;
        }

        double speed = 12; // inches per second
        drivesys.autoMecanumUntil(angle, speed, distance,colorIDs, rli, this);
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

        drivesys.PIDLineFollow(lfPID, 24, 12, this.ods, this.touchSensor);
    }

    public void step4_moveBackToMountainBase(AWDMecanumDS drivesys) throws InterruptedException {
        // If in blue alliance, move back in ~ 315 angle for ~ 36 inches
        // Else, move back in ~ 225 angle for ~ 36 inches

        // If in blue alliance, spin right 90 degrees
        // Else, spin left 90 degrees

        // Fold the sensor arm again, as it will come in the way climbing the mountian.
        colorServo.setPosition(0); // Need to test multiple values to find the correct one.

        // Move forward ~ 36 inches
        drivesys.autoMecanum(90, 36, 12, 0);
    }
}
