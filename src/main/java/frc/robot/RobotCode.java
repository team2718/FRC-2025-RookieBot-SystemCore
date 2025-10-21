package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotCode {

    PowerDistribution hub = new PowerDistribution(0);

    XboxController driverXboxController = new XboxController(0);

    SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    SparkMax leftMotorArm = new SparkMax(0, 10, MotorType.kBrushless);
    SparkMax rightMotorArm = new SparkMax(0, 11, MotorType.kBrushless);
    SparkMax topMotorIntake = new SparkMax(0, 12, MotorType.kBrushless);
    SparkMax bottomMotorIntake = new SparkMax(0, 13, MotorType.kBrushless);

    ProfiledPIDController armPIDController = new ProfiledPIDController(0.3, 0, 0,
            new Constraints(60, 50));

    ArmFeedforward armFeedforward = new ArmFeedforward(0.0, 0.9, 0.0);

    boolean aPressed = false;
    boolean bPressed = false;
    double actionTimer = 0;

    public void init() {

        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.smartCurrentLimit(40);
        armConfig.idleMode(IdleMode.kBrake);

        rightMotorArm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armConfig.absoluteEncoder.zeroCentered(true);
        armConfig.absoluteEncoder.zeroOffset(0.948);
        armConfig.inverted(true);

        leftMotorArm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.smartCurrentLimit(30);
        intakeConfig.idleMode(IdleMode.kBrake);

        topMotorIntake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bottomMotorIntake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /*
     * Moves the arm up if the right trigger is pressed, moves it down if the left
     * trigger is pressed.
     * If the arm goes past a certain point, you can't move if further, but you can
     * move it back.
     * 
     * Runs intake while the right bumper is pressed, runs outtake while the left
     * bumper is pressed
     */

    // remeber to program hanging! X & Y Buttons are set aside for 'move to drop
    // off'&'move to collection'

    public double getArmPositionDegrees() {
        return leftMotorArm.getAbsoluteEncoder().getPosition() * 360;
    }

    public void periodic() {

        if (DriverStation.isAutonomous()) {
            swerveSubsystem.setDesiredSpeeds(0.3, 0, 0);

            armPIDController.setGoal(0);

            double ffVoltage = armFeedforward.calculate(armPIDController.getSetpoint().position + 90,
            armPIDController.getSetpoint().velocity);
            double pidVoltage = armPIDController.calculate(getArmPositionDegrees());
            double commandedVoltage = ffVoltage + pidVoltage;

            commandedVoltage = Math.max(-7, Math.min(7, commandedVoltage));

            leftMotorArm.setVoltage(-commandedVoltage);
            rightMotorArm.setVoltage(-commandedVoltage);
            return;
        }

        SmartDashboard.putNumber("Encoder Arm Position", getArmPositionDegrees());
        SmartDashboard.putNumber("Goal Arm Position ", armPIDController.getSetpoint().position);

        swerveSubsystem.setDesiredSpeeds(driverXboxController.getLeftY(), driverXboxController.getLeftX(),
                2.0 * driverXboxController.getRightX());

        // if (!aPressed && !bPressed) {
        // if (driverXboxController.getRightTriggerAxis() > 0.1
        // && getArmPositionDegrees() < 80) {
        // leftMotorArm.set(-0.1);
        // rightMotorArm.set(-0.1);
        // } else if (driverXboxController.getLeftTriggerAxis() > 0.1
        // && getArmPositionDegrees() > -80) {
        // leftMotorArm.set(0.1);
        // rightMotorArm.set(0.1);
        // } else {
        // leftMotorArm.set(0);
        // rightMotorArm.set(0);
        // }
        // }

        armPIDController.setGoal(0);

        // Ground Coral
        if (driverXboxController.getAButton()) {
            armPIDController.setGoal(85);
        }
        // Algae Removal
        if (driverXboxController.getBButton()) {
            armPIDController.setGoal(40);
        }

        // Algae Removal
        if (driverXboxController.getYButton()) {
            armPIDController.setGoal(-30);
        }

        double ffVoltage = armFeedforward.calculate(armPIDController.getSetpoint().position + 90,
                armPIDController.getSetpoint().velocity);
        double pidVoltage = armPIDController.calculate(getArmPositionDegrees());
        double commandedVoltage = ffVoltage + pidVoltage;

        commandedVoltage = Math.max(-7, Math.min(7, commandedVoltage));

        leftMotorArm.setVoltage(-commandedVoltage);
        rightMotorArm.setVoltage(-commandedVoltage);

        // End Effector / Intake
        if (driverXboxController.getLeftBumperButton()) {
            topMotorIntake.set(0.7);
            bottomMotorIntake.set(0.7);
        } else if (driverXboxController.getRightBumperButton()) {
            topMotorIntake.set(-0.7);
            bottomMotorIntake.set(-0.7);
        } else {
            topMotorIntake.set(0);
            bottomMotorIntake.set(0);
        }

        // if (!aPressed && !bPressed) {
        // if (driverXboxController.getAButtonPressed()) {
        // // If A is pressed and a cycle isn't running, start the A cycle
        // // aPressed = true;
        // actionTimer = 0;
        // } else if (driverXboxController.getBButtonPressed()) {
        // // If B is pressed and a cycle isn't running, start the B cycle
        // // bPressed = true;
        // actionTimer = 0;
        // }
        // }

        // if (aPressed) {
        // if (leftMotorArm.getAbsoluteEncoder().getPosition() > 0) {
        // // Lower the arm to bottom
        // leftMotorArm.set(-0.5);
        // rightMotorArm.set(-0.5);
        // } else if (actionTimer < 20) {
        // // For one second, spin the intake wheels
        // leftMotorArm.set(0);
        // rightMotorArm.set(0);
        // actionTimer++;
        // topMotorIntake.set(0.5);
        // bottomMotorIntake.set(-0.5);
        // } else {
        // // Stop motors and end cycle
        // topMotorIntake.set(0);
        // bottomMotorIntake.set(0);
        // leftMotorArm.set(0);
        // rightMotorArm.set(0);
        // actionTimer = 0;
        // aPressed = false;
        // }
        // }

        // if (bPressed) {
        // if (leftMotorArm.getAbsoluteEncoder().getPosition() < 80) {
        // // Raise the arm to 80 degrees
        // leftMotorArm.set(0.5);
        // rightMotorArm.set(0.5);
        // } else if (actionTimer < 30) {
        // // Wait 1.5 seconds
        // leftMotorArm.set(0);
        // rightMotorArm.set(0);
        // actionTimer++;
        // } else if (actionTimer < 50) {
        // // Spin outtake wheels for one second
        // leftMotorArm.set(0);
        // rightMotorArm.set(0);
        // actionTimer++;
        // topMotorIntake.set(-0.5);
        // bottomMotorIntake.set(0.5);
        // } else {
        // // Stop motors and end cycle
        // leftMotorArm.set(0); // These ones on the arm motors seem a little redundant
        // rightMotorArm.set(0);
        // topMotorIntake.set(0);
        // bottomMotorIntake.set(0);
        // actionTimer = 0;
        // bPressed = false;
        // }
        // }
    }
}
