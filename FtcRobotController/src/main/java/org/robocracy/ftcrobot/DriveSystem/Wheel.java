package org.robocracy.ftcrobot.DriveSystem;

enum WheelType  {Mecanum, Omni, Tetrix};

/**
 * @author Team Robocracy
 */
public abstract class Wheel {
    double frictionCoeffMatStraight;
    double frictionCoeffMatStrafe;
//    double frictionCoefficientMountain;
    double diameter, circumference;
    WheelType wheelType;

        public Wheel(WheelType wheelType, double diameter, double coeffMatStraight, double coeffMatStrafe) {
            this.wheelType = wheelType;
            this.diameter = diameter;
            this.circumference = Math.PI * diameter;
            this.frictionCoeffMatStraight = coeffMatStraight;
            this.frictionCoeffMatStrafe = coeffMatStrafe;
        }
}
