package org.robocracy.ftcrobot;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

import org.robocracy.ftcrobot.DriveSystem.AWDMecanumDS;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.DriverStation.DriverStation;


/**
 * @author Team Robocracy
 * {@docRoot}
 *
 * Top level class in hierarchy. Represents an {@code FTCRobot} with main {@link FTCRobot#runRobotAutonomous(String)} and {@link FTCRobot#runRobotTeleop()} methods, which are used in {@link com.qualcomm.ftcrobotcontroller.opmodes.AutonomousRedNew}, {@link com.qualcomm.ftcrobotcontroller.opmodes.AutonomousBlueNew}, and {@link com.qualcomm.ftcrobotcontroller.opmodes.TeleOpNew} opmodes.
 */
public class FTCRobot {
    LinearOpMode curOpmode;
    public AWDMecanumDS driveSys;
    DeviceInterfaceModule dim;
    Harvester harvester;
    LinearLift linearLift;
    DcMotor harvesterMotor;
    AutonomousScorer autoScorer;
    Servo rightLatch;
    Servo leftLatch;
    //Servo bucketServo;
    Servo rightClimberServo;
    Servo leftClimberServo;
    Latch latch;
    Bucket bucket;
    LeftClimber leftClimber;
    RightClimber rightClimber;
    // RobotLength = Distance in inches from the center of front left to the center of rear left wheel
    double RobotLength;
    // RobotWidth = Distance in inches from the center of front left to the center of front right wheel
    double RobotWidth;
    public long timestamp;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    DriverStation drvrStation;

    public final int NAVX_DIM_I2C_PORT = 5;
    public AHRS navx_device;

    public FTCRobot(LinearOpMode curOpmode, boolean allianceIsBlue) {
        this.curOpmode = curOpmode;
        this.dim = curOpmode.hardwareMap.deviceInterfaceModule.get("dim");
        this.drvrStation = new DriverStation(curOpmode, this);
        this.harvesterMotor = curOpmode.hardwareMap.dcMotor.get("harvesterMotor");
        this.harvester = new Harvester(this, curOpmode, harvesterMotor);
        this.linearLift = new LinearLift(this, curOpmode);
        this.autoScorer = new AutonomousScorer(this, curOpmode, allianceIsBlue);
        this.navx_device = AHRS.getInstance(curOpmode.hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT, AHRS.DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);
        this.driveSys = new AWDMecanumDS(curOpmode, this);
        this.timestamp = System.nanoTime();
        this.leftLatch = curOpmode.hardwareMap.servo.get("leftLatch");
        this.rightLatch = curOpmode.hardwareMap.servo.get("rightLatch");
        //this.bucketServo = curOpmode.hardwareMap.servo.get("bucketServo");
        this.latch = new Latch(this, leftLatch, rightLatch, curOpmode);
        //this.bucket = new Bucket(this, curOpmode, bucketServo);
        this.rightClimberServo = curOpmode.hardwareMap.servo.get("rightClimber");
        this.leftClimberServo = curOpmode.hardwareMap.servo.get("leftClimber");
        this.leftClimber = new LeftClimber(this, leftClimberServo, curOpmode);
        this.rightClimber = new RightClimber(this, rightClimberServo, curOpmode);
    }

    public void runRobotAutonomous()  throws InterruptedException {

        this.autoScorer.step1_driveToRepairZone(this.driveSys);
        //this.autoScorer.step2_alignWithWhiteLine(this.driveSys);
        //this.autoScorer.step3_moveToTheRescueBeacon(this.driveSys);
        //this.autoScorer.step4_moveBackToMountainBase(this.driveSys);
/*
        this.driveSys.autoMecanum(250, 82, 12, 0);
        this.driveSys.autoMecanum(0, 0, 12, 70);
*/
    }

    /**
     * Runs Autonomous mode with values from file in {@code filePath}.
     * @param filePath file path of file with values to be read for robot to move
     * @throws InterruptedException
     */
    public void runRobotAutonomous(String filePath) throws InterruptedException {
        DbgLog.msg(String.format("Filename = %s", filePath));
        autoScorer.driveUsingReplay(filePath);
    }

    /**
     * Runs TeleOp mode by {@link DriverStation#getNextCommand()} for getting gamepad values.
     * @throws InterruptedException
     */
    public  void  runRobotTeleop() throws InterruptedException {
        DriverCommand driverCommand;
        while(curOpmode.opModeIsActive()){
            driverCommand = drvrStation.getNextCommand();
            this.driveSys.applyCmd(driverCommand);

            this.harvester.applyDSCmd(driverCommand);
            this.linearLift.applyCmd(driverCommand);

            this.latch.applyDSCmd(driverCommand);
            //this.bucket.applyDSCmd(driverCommand);
            this.leftClimber.applyDSCmd(driverCommand);
            this.rightClimber.applyDSCmd(driverCommand);

            // Wait for one hardware cycle for the setPower(0) to take effect.
            this.curOpmode.waitForNextHardwareCycle();

        }
    }
}
