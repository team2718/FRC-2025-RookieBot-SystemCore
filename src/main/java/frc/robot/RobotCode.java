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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotCode {

    PowerDistribution pdh = new PowerDistribution(0);

    XboxController driverXboxController = new XboxController(0);

    SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    SparkMax leftMotorArm = new SparkMax(0, 10, MotorType.kBrushless);
    SparkMax rightMotorArm = new SparkMax(0, 11, MotorType.kBrushless);
    SparkMax topMotorIntake = new SparkMax(0, 12, MotorType.kBrushless);
    SparkMax bottomMotorIntake = new SparkMax(0, 13, MotorType.kBrushless);

    ProfiledPIDController armPIDController = new ProfiledPIDController(0.3, 0.0, 0.00,
            new Constraints(100, 70));

    ArmFeedforward armFeedforward = new ArmFeedforward(0.0, 0.36, 0.09);

    boolean aPressed = false;
    boolean bPressed = false;
    double actionTimer = 0;

    String selectedAuto = Robot.kMoveAuto;

    private final Timer matchTimer = new Timer();

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

        matchTimer.reset();

        // Start match time on autonomous start
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> {
            matchTimer.reset();
            matchTimer.start();
        }));

        // Start match time on teleop start
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
            matchTimer.reset();
            matchTimer.start();
        }));

        // Stop match time on end of match
        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
            matchTimer.stop();
            matchTimer.reset();
        }));
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

        if (matchTimer.isRunning() && DriverStation.isDisabled()) {
            matchTimer.stop();
        }

        if (DriverStation.isDisabled()) {
            armPIDController.reset(getArmPositionDegrees());
        }

        SmartDashboard.putNumber("Encoder Arm Position", getArmPositionDegrees());
        SmartDashboard.putNumber("Goal Arm Position ", armPIDController.getSetpoint().position);
        SmartDashboard.putNumber("PDH Total Current", pdh.getTotalCurrent());

        double[] armPositions = {getArmPositionDegrees(), armPIDController.getSetpoint().position};

        SmartDashboard.putNumberArray("Arm", armPositions);

        if (DriverStation.isAutonomous()) {
            SmartDashboard.putNumber("Match Time", (15) - matchTimer.get());
        }

        if (DriverStation.isTeleop()) {
            SmartDashboard.putNumber("Match Time", (2 * 60 + 15) - matchTimer.get());
        }

        runArmPIDLoop();

        //// Autonomous ////

        if (DriverStation.isAutonomous()) {
            if (selectedAuto.equals(Robot.kMoveAuto)) {
                if (matchTimer.get() < 3) {
                    swerveSubsystem.setDesiredSpeeds(0.3, 0, 0);
                } else {
                    swerveSubsystem.setDesiredSpeeds(0.0, 0, 0);
                }
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
        if (driverXboxController.getRightBumperButton() || driverXboxController.getRightTriggerAxis() > 0.2) {
            topMotorIntake.set(-0.7);
            bottomMotorIntake.set(-0.7);
        } else if (driverXboxController.getLeftBumperButton() || driverXboxController.getLeftTriggerAxis() > 0.2) {
            topMotorIntake.set(0.7);
            bottomMotorIntake.set(0.7);
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
        commandedVoltage = Math.max(-8, Math.min(8, commandedVoltage));

        SmartDashboard.putNumber("Arm Voltage", commandedVoltage);

        leftMotorArm.setVoltage(commandedVoltage);
        rightMotorArm.setVoltage(commandedVoltage);
    }

    public void setAuto(String m_autoSelected) {
        this.selectedAuto = m_autoSelected;
    }
}
