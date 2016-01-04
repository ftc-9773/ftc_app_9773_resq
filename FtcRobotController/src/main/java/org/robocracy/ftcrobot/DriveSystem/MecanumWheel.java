package org.robocracy.ftcrobot.DriveSystem;

/**
 * @author Team Robocracy
 */
public class MecanumWheel extends Wheel {
    public MecanumWheel(double diameter, double frictionCoeffMatStraight, double frictionCoeffMatStrafe){
        super(WheelType.Mecanum, diameter, frictionCoeffMatStraight, frictionCoeffMatStrafe);
    }
}
