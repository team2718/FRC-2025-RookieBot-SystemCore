package frc.robot;

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

    String selectedAuto = Robot.kMoveAuto;

    public void init() {

        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.smartCurrentLimit(40);
        armConfig.idleMode(IdleMode.kBrake);
        armConfig.inverted(true);

        rightMotorArm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armConfig.absoluteEncoder.zeroCentered(true);
        armConfig.absoluteEncoder.zeroOffset(0.948);
        armConfig.inverted(false);

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

        //// Always run these ////

        SmartDashboard.putNumber("Encoder Arm Position", getArmPositionDegrees());
        SmartDashboard.putNumber("Goal Arm Position ", armPIDController.getSetpoint().position);

        runArmPIDLoop();

        //// Autonomous ////

        if (DriverStation.isAutonomous()) {
            if (selectedAuto.equals(Robot.kMoveAuto)) {
                swerveSubsystem.setDesiredSpeeds(0.3, 0, 0);
            }

            if (selectedAuto.equals(Robot.kStillAuto)) {
                swerveSubsystem.setDesiredSpeeds(0.0, 0, 0);
            }

            // Always keep arm up and out of the way in auto
            armPIDController.setGoal(0);
            return;
        }

        //// Teleop ////

        // Swerve driver control
        swerveSubsystem.setDesiredSpeeds(driverXboxController.getLeftY(), driverXboxController.getLeftX(),
                2.0 * driverXboxController.getRightX());

        // Default to arm up
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
    }

    private void runArmPIDLoop() {

        // +90 is an offset to account for us treating arm up as 0deg but the feed
        // forward model needing arm up to be 90.
        double ffVoltage = armFeedforward.calculate(armPIDController.getSetpoint().position + 90,
                armPIDController.getSetpoint().velocity);
        double pidVoltage = armPIDController.calculate(getArmPositionDegrees());
        double commandedVoltage = ffVoltage + pidVoltage;

        // Clamp voltage
        commandedVoltage = Math.max(-9, Math.min(9, commandedVoltage));

        leftMotorArm.setVoltage(commandedVoltage);
        rightMotorArm.setVoltage(commandedVoltage);
    }

    public void setAuto(String m_autoSelected) {
        this.selectedAuto = m_autoSelected;
    }
}
