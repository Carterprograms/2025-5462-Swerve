package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * Class to represent and handle a swerve module
 * A module's state is measured by a CANCoder for the absolute position, integrated CANEncoder for relative position
 * for both steeration and linear movement
 */
public class SwerveModule extends SubsystemBase {
    
    // Instanciate Motors
    private final SparkFlex driveMtr;
    private final SparkFlex steerMtr;

    // Instanciate Encoders
    private final RelativeEncoder driveEnc;
    private final RelativeEncoder steerEnc;
    private final CANcoder canCoder;

    // Instanciate PID Controllers
    private final SparkClosedLoopController steerController;
    private final SparkClosedLoopController driveController;

    public SwerveModule(int driveMtrId, int steerMtrId, int canCoderId, Rotation2d offset) {

        // Create Motors
        driveMtr = new SparkFlex(driveMtrId, MotorType.kBrushless);
        steerMtr = new SparkFlex(steerMtrId, MotorType.kBrushless);

        // Create Encoders
        driveEnc = driveMtr.getEncoder();
        steerEnc = steerMtr.getEncoder();
        canCoder = new CANcoder(canCoderId);

        // Create PID Controllers
        steerController = steerMtr.getClosedLoopController();
        driveController = driveMtr.getClosedLoopController();

        // Drive Spark Flex Configs
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        driveConfig.encoder
            .positionConversionFactor(DriveConstants.driveMetersPerEncRev)
            .velocityConversionFactor(DriveConstants.driveMetersPerSecPerMtrRPM);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(DriveConstants.drivekP, 0, DriveConstants.drivekD);

        // Steer Spark Flex Configs
        SparkFlexConfig steerConfig = new SparkFlexConfig();
        steerConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        steerConfig.encoder
            .positionConversionFactor(DriveConstants.steerRadiansPerEncRev);
        steerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(DriveConstants.steerkP, 0, DriveConstants.steerkD);

        // Apply Configs
        driveMtr.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerMtr.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Can-Encoder Config
        CANcoderConfiguration ccConfig = new CANcoderConfiguration();
        ccConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoder.getConfigurator().apply(ccConfig);

        // Initializes the steer encoder position to the CANCoder position, accounting for offset.
        steerEnc.setPosition(getCanCoderAngle().getRadians());
    }

    /**
     * Returns the current position of the module.
     * 
     * @return The current position of the module
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveEnc.getPosition(), getSteerEncAngle()
        );
    }

    /**
     * Resets the distance travled bt the module to zero.
     */
    public void resetDriveDistance() {
        driveEnc.setPosition(0.0);
    }

    /**
     * Returns the current drive distance of the module.
     * 
     * @return The current drive distance of the module.
     */
    public double getDriveDistanceMeters() {
        return driveEnc.getPosition();
    }

    /**
     * Returns the current absolute angle of the module from the CANCoder.
     * This measurment does not account for offset.
     * It is preferred to use the method getSteerEncAngle().
     * 
     * @return The value of the CANCoder.
     */
    public Rotation2d getCanCoderAngle() {
        return new Rotation2d(canCoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI);
    }

    /**
     * Returns the current absolute angle of the module from the steer motor encoder.
     * This measurment accounts for offset.
     * 
     * @return The current absolute angle of the module.
     */
    public Rotation2d getSteerEncAngle() {
        return new Rotation2d(steerEnc.getPosition());
    }

    /**
     * Returns the current velocity of the module from the drive motor encoder.
     * 
     * @return The current velocity of the module in meters per second.
     */
    public double getVelocityMetersPerSec() {
        return driveEnc.getVelocity();
    }

    /**
     * Calculates the angle motor setpoint based on the desired angle and the current angle measurment.
     * 
     * @param targetAngle The desired angle to set the module in radians.
     * @param currentAngle The current angle of the module in radians.
     * 
     * @return The adjusted target angle for the module in radians.
     */
    public double calculatedAdjustedAngle(double targetAngle, double currentAngle) {
        double modAngle = currentAngle % (2.0 * Math.PI);

        if (modAngle < 0.0) modAngle += 2.0 * Math.PI;

        double newTarget = targetAngle + currentAngle - modAngle;

        if (targetAngle - modAngle > Math.PI) newTarget -= 2.0 * Math.PI;
        else if (targetAngle - modAngle < -Math.PI) newTarget += 2.0 * Math.PI;

        return newTarget;
    }

    /**
     * Sets the desired state of the swerve module and optimizes it.
     * <p>If closed-loop, uses PID and a feedforward to control the speed.
     * If open-loop, sets the speed to a percentage. Open-loop control sould
     * only be used if running an autonomour trajectory.
     * 
     * @param desiredState Object that holds a desired linear and steerational setpoint.
     * @param isClosedLoop True if the velocity control is a closed-loop.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isClosedLoop) {

        // Calculate the current angle of the module
        double currentAngle = getSteerEncAngle().getRadians();

        // Optimize the desired state to the current angle
        double optimizedAngle = calculatedAdjustedAngle(desiredState.angle.getRadians(), currentAngle);

        // Upadte the desired state with the optimized angle
        desiredState = new SwerveModuleState(desiredState.speedMetersPerSecond, new Rotation2d(optimizedAngle));

        // Scale velocity based on turn error to help prevent skew.
        double angleErrorRad = desiredState.angle.getRadians() - getSteerEncAngle().getRadians();
        desiredState.speedMetersPerSecond *= Math.cos(angleErrorRad);

        steerController.setReference(
            calculatedAdjustedAngle(
                desiredState.angle.getRadians(),
                getSteerEncAngle().getRadians()),
            ControlType.kPosition
        );

        if(!isClosedLoop) {
            driveMtr.set(desiredState.speedMetersPerSecond / DriveConstants.freeMetersPerSecond);
        }
        else {
            driveController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity
            );
        }
    }

    public void runCharacterization(double volts) {
        steerController.setReference(
            0.0,
            ControlType.kPosition
        );

        driveMtr.setVoltage(volts);
    }

    public double getDriveVoltage() {
        return driveMtr.get() * 12.0;
    }

}
