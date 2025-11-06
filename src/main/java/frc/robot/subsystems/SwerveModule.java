package frc.robot.subsystems;

import java.io.FileWriter;
import java.io.IOException;

import org.json.simple.JSONObject;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
    private final PIDController steerController;
    private final PIDController driveController;

    // Instanciate Spark Flex simulation
    private final SparkFlexSim driveFlexSim;
    private final SparkFlexSim steerFlexSim;

    // Instanciate motor simulation
    public DCMotorSim driveSim;
    public DCMotorSim steerSim;

    public SwerveModule(int driveMtrId, int steerMtrId, int canCoderId, Rotation2d offset) {

        // Create Motors
        driveMtr = new SparkFlex(driveMtrId, MotorType.kBrushless);
        steerMtr = new SparkFlex(steerMtrId, MotorType.kBrushless);

        // Create Encoders
        driveEnc = driveMtr.getEncoder();
        steerEnc = steerMtr.getEncoder();
        canCoder = new CANcoder(canCoderId);

        // Create PID Controllers
        steerController = new PIDController(0.1, 0, 0);
        driveController = new PIDController(0.1, 0, 0);

        // Drive Spark Flex Configs
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast);
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
        steerEnc.setPosition(getCanCoderAngle().getRadians() - offset.getRadians());

        // Initialize Spark Flex simulation
        driveFlexSim = new SparkFlexSim(driveMtr, DCMotor.getNeoVortex(1));
        steerFlexSim = new SparkFlexSim(steerMtr, DCMotor.getNeoVortex(1));

        // Initalize motor simulation
        DCMotor gearbox = DCMotor.getNeoVortex(1);
        LinearSystem<N2, N1, N2> dcMotorPlantDrive =
            LinearSystemId.createDCMotorSystem(
                DriveConstants.kvVoltsSecsPerRad,
                DriveConstants.kaVoltSecsPerRadSq);

        LinearSystem<N2, N1, N2> dcMotorPlantSteer = LinearSystemId.createDCMotorSystem(
            1 / DCMotor.getNeoVortex(1).KvRadPerSecPerVolt * DriveConstants.steerMtrGearReduction,
            0.01 
        );

        steerSim = new DCMotorSim(dcMotorPlantSteer, gearbox);
        driveSim = new DCMotorSim(dcMotorPlantDrive, gearbox);
    }

    public double getSimDriveVoltage() {
        return driveFlexSim.getAppliedOutput() * 12.0;
    }

    @Override
    public void periodic() {
        // Set Inputs
        steerSim.setInputVoltage(steerFlexSim.getAppliedOutput() * 12.0);
        driveSim.setInputVoltage(driveFlexSim.getAppliedOutput() * 12.0);

        // Update sims
        driveSim.update(0.02);
        steerSim.update(0.02);

        steerFlexSim.iterate((steerSim.getAngularVelocityRPM() / 60.0), 12, 0.02);

        driveFlexSim.iterate((driveSim.getAngularVelocityRPM() / 60.0) * DriveConstants.wheelCircumferenceMeters, 12, 0.02);

        canCoder.getSimState().setRawPosition(steerSim.getAngularPositionRotations());

        double steerSimPose = steerSim.getAngularPositionRotations();
        double driveSimVelocity = (driveSim.getAngularVelocityRPM() / 60) * DriveConstants.wheelCircumferenceMeters;
        double driveSimPose = driveSim.getAngularPositionRotations() * DriveConstants.wheelCircumferenceMeters;

        steerFlexSim.setPosition(steerSimPose);
        driveFlexSim.setVelocity(driveSimVelocity);
        driveFlexSim.setPosition(driveSimPose);

        SmartDashboard.putNumber("driveMotor/id" + driveMtr.getDeviceId() + "/simPos", driveSim.getAngularPositionRad());
        SmartDashboard.putNumber("steerMotor/id" + steerMtr.getDeviceId() + "/simPos", steerSim.getAngularPositionRad());


        // Log velocity and voltage for characterization
        LogStufftoJSON(getVelocityMetersPerSec(), getSimDriveVoltage(), getDriveCurrent(), getCanCoderAngle(), getSteerEncAngle()); // CHANGE TO getDriveVoltage() FOR REAL ROBOT!!!!!!!!!
    }

/**
 * Logs the velocity and voltage of the robot to the JSON file
 * 
 * @param driveVelocity The average drive velocity of the robot in meters per second.
 * @param voltage The average drive voltage of the robot in volts.
 * @param current The average drive current of the robot in amps.
 */
@SuppressWarnings("unchecked")
private void LogStufftoJSON(double driveVelocity, double voltage, double current, Rotation2d canCoderAngle, Rotation2d steerEncAngle) {
    try {
        // Create a JSON object with the velocity and voltage
        JSONObject jsonObject = new JSONObject();
        jsonObject.put("timestamp", System.currentTimeMillis());
        jsonObject.put("driveVelocity", driveVelocity);
        jsonObject.put("voltage", voltage);
        jsonObject.put("current", current);
        jsonObject.put("canCoderAngleDegrees", canCoderAngle.getDegrees()); // Convert to degrees
        jsonObject.put("steerEncAngleDegrees", steerEncAngle.getDegrees()); // Convert to degrees

        // Write the JSON object to the file
        try (FileWriter file = new FileWriter("logging/velocity-voltage_log.json", true)) {
            file.write(jsonObject.toString() + "\n");
        }
    } catch (IOException e) {
        System.err.println("Error writing to Log file: " + e.getMessage());
    }
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
     * Returns the average current draw of the drive motors.
     * 
     * @return The average current draw of the drive motors.
     */
    public double getDriveCurrent() {
        return driveMtr.getOutputCurrent();
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
        return new Rotation2d(canCoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI);
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

        // Ensure the desired state is optimized for the current angle.
        desiredState.optimize(getCanCoderAngle());

        // Scale velocity based on turn error to help prevent skew.
        double angleErrorRad = desiredState.angle.getRadians() - getSteerEncAngle().getRadians();
        desiredState.speedMetersPerSecond *= Math.cos(angleErrorRad);

        // Log values for debugging
        //System.out.println("Raw Encoder Value: " + canCoder.getAbsolutePosition().getValueAsDouble());
        //System.out.println("Steer Encoder Angle: " + getSteerEncAngle().getRadians());

        double steerControl = steerController.calculate(getSteerEncAngle().getRadians(), desiredState.angle.getRadians());
        steerMtr.set(steerControl);

        if(!isClosedLoop) {
            driveMtr.set(desiredState.speedMetersPerSecond / DriveConstants.freeMetersPerSecond);
        }
        else {
            driveController.calculate(getVelocityMetersPerSec(), desiredState.speedMetersPerSecond);
        }
    }

    public void runCharacterization(double volts) {
        steerController.calculate(getSteerEncAngle().getRadians(), 0.0);

        driveMtr.setVoltage(volts);
    }

    public double getDriveVoltage() {
        return driveMtr.get() * 12.0;
    }

}