package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;

public class RobotCode {

    XboxController driverXboxController = new XboxController(0);

    SparkMax leftMotorArm = new SparkMax(6, MotorType.kBrushless);
    SparkMax rightMotorArm = new SparkMax(7, MotorType.kBrushless);
    SparkMax topMotorIntake = new SparkMax(8, MotorType.kBrushless);
    SparkMax bottomMotorIntake = new SparkMax(9, MotorType.kBrushless);

    boolean aPressed = false;
    boolean bPressed = false;
    double actionTimer = 0;

    public void init() {
        
    }

/* Moves the arm up if the right trigger is pressed, moves it down if the left trigger is pressed. 
If the arm goes past a certain point, you can't move if further, but you can move it back.

Runs intake while the right bumper is pressed, runs outtake while the left bumper is pressed
*/

//remeber to program hanging! X & Y Buttons are set aside for 'move to drop off'&'move to collection'
    public void periodic() {
        if (!aPressed && !bPressed){
            if (driverXboxController.getRightTriggerAxis() > 0.1 && leftMotorArm.getAbsoluteEncoder().getPosition() < 120) {
                leftMotorArm.set(0.5);
                rightMotorArm.set(0.5);
            }
            else if (driverXboxController.getLeftTriggerAxis() > 0.1 && leftMotorArm.getAbsoluteEncoder().getPosition() > 0){
                leftMotorArm.set(-0.5);
                rightMotorArm.set(-0.5);
            }
            else {
                leftMotorArm.set(0);
                rightMotorArm.set(0);
            }

            if (driverXboxController.getRightBumperButtonPressed()){
                topMotorIntake.set(0.5);
                bottomMotorIntake.set(-0.5);
            }
            else if (driverXboxController.getLeftBumperButtonPressed()){
                topMotorIntake.set(-0.5);
                bottomMotorIntake.set(0.5);
            }
            else {
                topMotorIntake.set(0);
                bottomMotorIntake.set(0);
            }
        }

        if (!aPressed && !bPressed){
            if (driverXboxController.getAButtonPressed()){
                //If A is pressed and a cycle isn't running, start the A cycle
                //aPressed = true;
                actionTimer = 0;
            }
            else if (driverXboxController.getBButtonPressed()){
                //If B is pressed and a cycle isn't running, start the B cycle
                //bPressed = true;
                actionTimer = 0;
            }
        }

        if (aPressed){
            if (leftMotorArm.getAbsoluteEncoder().getPosition() > 0){
                //Lower the arm to bottom
                leftMotorArm.set(-0.5);
                rightMotorArm.set(-0.5);
            }
            else if (actionTimer < 20) {
                //For one second, spin the intake wheels
                leftMotorArm.set(0);
                rightMotorArm.set(0);
                actionTimer++;
                topMotorIntake.set(0.5);
                bottomMotorIntake.set(-0.5);
            }
            else {
                //Stop motors and end cycle
                topMotorIntake.set(0);
                bottomMotorIntake.set(0);
                leftMotorArm.set(0);
                rightMotorArm.set(0);
                actionTimer = 0;
                aPressed = false;
            }
        }

        if (bPressed){
            if (leftMotorArm.getAbsoluteEncoder().getPosition() < 80){
                //Raise the arm to 80 degrees
                leftMotorArm.set(0.5);
                rightMotorArm.set(0.5);
            }
            else if (actionTimer < 30){
                //Wait 1.5 seconds
                leftMotorArm.set(0);
                rightMotorArm.set(0);
                actionTimer++;
            }
            else if (actionTimer < 50){
                //Spin outtake wheels for one second
                leftMotorArm.set(0);
                rightMotorArm.set(0);
                actionTimer++;
                topMotorIntake.set(-0.5);
                bottomMotorIntake.set(0.5);
            }
            else {
                //Stop motors and end cycle
                leftMotorArm.set(0); //These ones on the arm motors seem a little redundant
                rightMotorArm.set(0);
                topMotorIntake.set(0);
                bottomMotorIntake.set(0);
                actionTimer = 0;
                bPressed = false;
            }
        }
    }
}
