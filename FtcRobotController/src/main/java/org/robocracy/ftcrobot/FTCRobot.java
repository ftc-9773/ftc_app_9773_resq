package org.robocracy.ftcrobot;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.opmodes.AutonomousBlue;
import com.qualcomm.ftcrobotcontroller.opmodes.AutonomousRed;
import com.qualcomm.ftcrobotcontroller.opmodes.TeleOpBlue;
import com.qualcomm.ftcrobotcontroller.opmodes.TeleOpRed;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.robocracy.ftcrobot.DriveSystem.AWDMecanumDS;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.util.FileRW;
import org.robocracy.ftcrobot.util.NavX;
import org.robocracy.ftcrobot.util.UserHandler;

import java.io.File;
import java.io.IOException;

import javax.xml.parsers.SAXParser;
import javax.xml.parsers.SAXParserFactory;
import org.xml.sax.helpers.DefaultHandler;


/**
 * Top level class in hierarchy. Represents an {@code FTCRobot} with
 *     main {@link FTCRobot#runRobotAutonomous()} and {@link FTCRobot#runRobotTeleop()} methods,
 *     which are used in {@link AutonomousRed}, {@link AutonomousBlue}, {@link TeleOpBlue} and {@link TeleOpRed} opmodes.
 * @author Team Robocracy
 * {@docRoot}
 */
public class FTCRobot {
    LinearOpMode curOpmode;
    public AWDMecanumDS driveSys;
    DeviceInterfaceModule dim = null;
    Harvester harvester = null;
    LinearLift linearLift = null;
    DcMotor harvesterMotor = null;
    AutonomousScorer autoScorer;
    Servo rightLatch = null;
    Servo leftLatch = null;
    Servo bucketServo = null;
    Servo rightClimberServo = null;
    Servo leftClimberServo = null;
    Servo climberDispenserServo = null;
    Servo signalReleaseServo = null;
    Latch latch = null;
    Bucket bucket = null;
    LeftClimber leftClimber = null;
    RightClimber rightClimber = null;
    EndGamePlayer endGamePlayer = null;
    ClimberDispenser climberDispenser = null;
    SignalReleaser signalReleaser = null;
    public OpticalDistanceSensor ods = null;
    public ColorSensor colorSensor = null;
    public FileRW readFileRW, writeFileRW;
    File initFile;
    SAXParserFactory factory;
    SAXParser saxParser;
    UserHandler userHandler;

    public enum currentlyRecording{NONE, RECORDING_AUTONOMOUS, RECORDING_ENDGAME}
    public currentlyRecording curStatus = currentlyRecording.NONE;
    public long timestamp;
    // ToDo: NAVX_DEVICE_UPDATE_RATE_HZ may have to  be increased to 100
    //  Value of 50 implies a 20 milli second hardwsre cycle (50 * 20 = 1000)
    //  A value of 100 implies 10 millisecond hardware cycle, which is closer to the
    //  hardware cycle value we observed.
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    public NavX navx_device = null;
    public NavX.CollisionDetector collisionDetector = null;

    DriverStation drvrStation;

    public final int NAVX_DIM_I2C_PORT = 5;
    public AHRS navxDevice = null;

    public FTCRobot(LinearOpMode curOpmode, String readFilePath, String writeFilePath, boolean allianceIsBlue, currentlyRecording curStatus) throws InterruptedException {
        this.curOpmode = curOpmode;
        /*initDevice("dim");
        initDevice("ods");
        initDevice("colorSensor");
        initDevice("navx");
        initDevice("harvesterMotor");
        initDevice("leftLatch");
        initDevice("rightLatch");
        initDevice("leftClimber");
        initDevice("rightClimber");
        initDevice("bucketServo");
        initDevice("climberDispenserServo");
        initDevice("signalReleaseServo");

        if (this.navxDevice != null) {
            this.navx_device = new NavX(this, curOpmode, this.navxDevice);
            this.collisionDetector = new NavX.CollisionDetector(this.navxDevice);
        }
        this.drvrStation = new DriverStation(curOpmode, this);
        this.harvester = new Harvester(this, curOpmode, harvesterMotor);
        this.linearLift = new LinearLift(this, curOpmode);
        this.autoScorer = new AutonomousScorer(this, curOpmode, allianceIsBlue);
        this.driveSys = new AWDMecanumDS(curOpmode, this);
        this.latch = new Latch(this, leftLatch, rightLatch, curOpmode);
        this.bucket = new Bucket(this, curOpmode, bucketServo);
        this.leftClimber = new LeftClimber(this, leftClimberServo, curOpmode, allianceIsBlue);
        this.rightClimber = new RightClimber(this, rightClimberServo, curOpmode, allianceIsBlue);
        this.endGamePlayer = new EndGamePlayer(this, curOpmode, allianceIsBlue);
        this.climberDispenser = new ClimberDispenser(this, climberDispenserServo, curOpmode);
        this.signalReleaser = new SignalReleaser(this, signalReleaseServo, curOpmode);*/
        this.initFile = new File("/sdcard/FIRST/init.xml");
        this.factory = SAXParserFactory.newInstance();
        try {
            this.saxParser = this.factory.newSAXParser();
        }
        catch (Exception e){

        }
        this.userHandler = new UserHandler();
        saxParser.parse();
        this.curStatus = curStatus;

        if (readFilePath != null) {
            this.readFileRW = new FileRW(readFilePath, false);
            DbgLog.msg(String.format("readFilePath is %s", readFilePath));
        } else {
            this.readFileRW = null;
            DbgLog.msg(String.format("readFilePath is null"));
        }
        if (writeFilePath != null) {
            this.writeFileRW = new FileRW(writeFilePath, true);
            DbgLog.msg(String.format("writeFilePath is %s", writeFilePath));
        } else {
            this.writeFileRW = null;
            DbgLog.msg(String.format("writeFilePath is null"));
        }
    }

    void initDevice (String deviceName) {
        try {
            if (deviceName.matches("dim")) {
                this.dim = curOpmode.hardwareMap.deviceInterfaceModule.get("dim");
            } else if(deviceName.matches("ods")){
                this.ods = curOpmode.hardwareMap.opticalDistanceSensor.get("ods_sensor");
            } else if(deviceName.matches("colorSensor")){
                this.colorSensor = curOpmode.hardwareMap.colorSensor.get("color_sensor");
            } else if (deviceName.matches("navx") && (this.dim != null)) {
                this.navxDevice = AHRS.getInstance(this.dim,
                        NAVX_DIM_I2C_PORT, AHRS.DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);
            } else if (deviceName.matches("harvesterMotor")) {
                this.harvesterMotor = curOpmode.hardwareMap.dcMotor.get("harvesterMotor");
            } else if (deviceName.matches("leftLatch")) {
                this.leftLatch = curOpmode.hardwareMap.servo.get("leftLatch");
            } else if (deviceName.matches("rightLatch")) {
                this.rightLatch = curOpmode.hardwareMap.servo.get("rightLatch");
            } else if (deviceName.matches("leftClimber")) {
                this.leftClimberServo = curOpmode.hardwareMap.servo.get("leftClimber");
            } else if (deviceName.matches("rightClimber")) {
                this.rightClimberServo = curOpmode.hardwareMap.servo.get("rightClimber");
            } else if (deviceName.matches("bucketServo")) {
                this.bucketServo = curOpmode.hardwareMap.servo.get("bucketServo");
            } else if (deviceName.matches("climberDispenserServo")){
                this.climberDispenserServo = curOpmode.hardwareMap.servo.get("climberDispenserServo");
            } else if(deviceName.matches("signalReleaseServo")){
                this.signalReleaseServo = curOpmode.hardwareMap.servo.get("signalReleaseServo");
            }
        }
        catch(Exception e){
            DbgLog.error(String.format("%s . Device skipped", e.getMessage()));
        }
    }

    /**
     * Runs Autonomous mode with values from file in {@code filePath}.
     * @throws InterruptedException
     */
    public void runRobotAutonomous()  throws InterruptedException {
        float targetYaw;
        DriverCommand tmpDrvrCmd = new DriverCommand();

        autoScorer.driveUsingReplay();
        /*if (autoScorer.findTheWhiteLine()) {
            DbgLog.msg(String.format("Yay! Found the white tape!"));
            if (autoScorer.allianceIsBlue) {
                targetYaw = navx_device.initial_navx_data[0] - 90;
            } else {
                targetYaw = navx_device.initial_navx_data[0] + 90;
            }
            driveSys.rotateToAngle(targetYaw);
            DbgLog.msg("Now going Straight");
            driveSys.PIDmoveStraight(0.05, 0, 0, 270, 12);
            tmpDrvrCmd.climberDispenserCommand.climberDispenserStatus = -1;
            climberDispenser.applyDSCmd(tmpDrvrCmd);
            curOpmode.waitOneFullHardwareCycle();
            tmpDrvrCmd.climberDispenserCommand.climberDispenserStatus = -2;
            climberDispenser.applyDSCmd(tmpDrvrCmd);
            curOpmode.waitOneFullHardwareCycle();
        }*/
        try {
            if (readFileRW != null){
                readFileRW.close();
            }
            if (writeFileRW != null) {
                writeFileRW.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void runRobotAutonomous(int distanceToStrafe) throws InterruptedException {
        this.autoScorer.strafeTheDistance(driveSys, distanceToStrafe);
    }

    /**
     * Runs Teleop mode by {@link DriverStation#getNextCommand()} for getting gamepad values.
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
            this.bucket.applyDSCmd(driverCommand);
            this.leftClimber.applyDSCmd(driverCommand);
            this.rightClimber.applyDSCmd(driverCommand);
            this.climberDispenser.applyDSCmd(driverCommand);
            this.signalReleaser.applyDSCmd(driverCommand);
//            this.endGamePlayer.runEndGame(driverCommand);

            // Wait for one hardware cycle for the setPower(0) to take effect.
            this.curOpmode.waitForNextHardwareCycle();

        }
    }

    public void close() {
        this.driveSys.close();
        this.collisionDetector.close();
        this.navxDevice.close();
    }
}
