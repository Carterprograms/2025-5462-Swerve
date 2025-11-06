package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.limelight.LimelightPoseEstimator;

public class SwerveSys extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // Instantiates the Pigeon2 IMU
  private final Pigeon2 imu = new Pigeon2(CANDevices.imuId);


  // PathPlanner Config
  RobotConfig config;


  // Initializes swerve module objects
  private final SwerveModule frontLeftMod = 
  new SwerveModule(
      CANDevices.frontLeftDriveMtrId,
      CANDevices.frontLeftSteerMtrId,
      CANDevices.frontLeftCanCoderId,
      DriveConstants.frontLeftModOffset
  );

  private final SwerveModule frontRightMod =
    new SwerveModule(
      CANDevices.frontRightDriveMtrId,
      CANDevices.frontRightSteerMtrId,
      CANDevices.frontRightCanCoderId,
      DriveConstants.frontRightModOffset
    );

  private final SwerveModule backLeftMod = 
    new SwerveModule(
      CANDevices.backLeftDriveMtrId,
      CANDevices.backLeftSteerMtrId,
      CANDevices.backLeftCanCoderId,
      DriveConstants.backLeftModOffset
    );

  private final SwerveModule backRightMod =
    new SwerveModule(
      CANDevices.backRightDriveMtrId,
      CANDevices.backRightSteerMtrId,
      CANDevices.backRightCanCoderId,
      DriveConstants.backRightModOffset
    );

  private boolean isLocked = false;
  public boolean isLocked() {
    return isLocked;
  }

  public boolean isFiledOriented = true;
  public boolean isFieldOriented() {
    return isFiledOriented;
  }

  private double speedFactor = 1.0;
  public double getSpeedFactor() {
    return speedFactor;
  }

  /**
   * Sets the speed factor of the robot. Inputs are multiplied by the factor to reduce drive speed.
   * Useful for "Turtle" or "Sprint" modes.
   * @param speedFactor The factor to scale inputs, as a percentage.
   */
  public void setSpeedFactor(double speedFactor) {
    this.speedFactor = speedFactor;
  }

  private Optional<Double> omegaOverrideRadPerSec = Optional.empty();
  public void setOmegaOverrideRadPerSec(Optional<Double> omegaOverrideRadPerSec) {
    this.omegaOverrideRadPerSec = omegaOverrideRadPerSec;
  }

  public boolean hasOmegaOverride() {
    return omegaOverrideRadPerSec.isPresent();
  }

  private final SwerveDrivePoseEstimator poseEstimator =
    new SwerveDrivePoseEstimator(
      DriveConstants.kinematics, // Kinematics model for the swerve drive
      imu.getRotation2d(),       // Initial heading from the IMU
      getModulePositions(),      // Initial positions of the swerve modules
      getPose(),                 // Initial pose of the robot
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(0.25)),  // Measured State standard deviations
      VecBuilder.fill(0.35,0.35, Units.degreesToRadians(35.0))   // Measured Vision standard deviations
    );

  private final LimelightPoseEstimator[] limelightPoseEstimators = new LimelightPoseEstimator[] {
    new LimelightPoseEstimator(VisionConstants.limeLightName)
  };

  public void resetPPPose(Pose2d pose) {
    setPose(pose);
  }

  /**
   * Constructs a new SwerveSys.
   * 
   * <p>SwerveCmd contains 4 {@link SwerveModule}, a gyro, and methods to control the drive base and odometry.
   */
  public SwerveSys() {

    // Gets the robots configuration from Path Planner
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Resets the measured distance driven for each module
    frontLeftMod.resetDriveDistance();
    frontRightMod.resetDriveDistance();
    backLeftMod.resetDriveDistance();
    backRightMod.resetDriveDistance();

    resetPose();

    System.out.println(frontLeftMod.getSteerEncAngle());
    System.out.println(frontRightMod.getSteerEncAngle());
    System.out.println(backLeftMod.getSteerEncAngle());
    System.out.println(backRightMod.getSteerEncAngle());

    // Configure AutoBuilder last
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPPPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds,
      (speeds, feedforwards) -> setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionall outputs indiviual module feedforwards
      new PPHolonomicDriveController(
        new PIDConstants(1, 1, 1), // Translation PID constants
        new PIDConstants(1, 1, 1) // Rotation PID constants
      ),
      config, // PathPlanner robot configuration
      () -> {
        // Boolean supplier that controls that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        } 
          // Default to false if alliance is not set
          return false;
        },
        this // Refrence to this suybsystem to set requirements
    );
  }

  // Send simulation output to NetworkTables
  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();
  StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

  StructArrayPublisher<SwerveModuleState> measuredStatePublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("StatesMeasured", SwerveModuleState.struct).publish();

  StructArrayPublisher<SwerveModuleState> targetStatePublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("StatesTarget", SwerveModuleState.struct).publish();

  @Override
  public void periodic() {

    // Periodicaly send simulated pose and module states to NetworkTables

    SwerveModuleState[] measuredStates = new SwerveModuleState[] {
      new SwerveModuleState(frontLeftMod.getVelocityMetersPerSec(), frontLeftMod.getSteerEncAngle()),
      new SwerveModuleState(frontRightMod.getVelocityMetersPerSec(), frontRightMod.getSteerEncAngle()),
      new SwerveModuleState(backLeftMod.getVelocityMetersPerSec(), backLeftMod.getSteerEncAngle()),
      new SwerveModuleState(backRightMod.getVelocityMetersPerSec(), backRightMod.getSteerEncAngle())
    };

    Pose2d poseA = new Pose2d();
    Pose2d poseB = new Pose2d();

    poseA = new Pose2d(0.0, 0.0, imu.getRotation2d());
    poseB = getPose();

    measuredStatePublisher.set(measuredStates);
    publisher.set(poseA);
    arrayPublisher.set(new Pose2d[] {poseA, poseB});

    // This method will be called once per scheduler run

    poseEstimator.update(imu.getRotation2d(), getModulePositions());

      for(LimelightPoseEstimator limelightPoseEstimator : limelightPoseEstimators) {
      Optional<Pose2d> limelightPose = limelightPoseEstimator.getRobotPose();
      if(limelightPose.isPresent()) {
        poseEstimator.addVisionMeasurement(limelightPose.get(), limelightPoseEstimator.getCaptureTimestamp());
      }
    }

    System.out.println("Pose" + getPose());
    System.out.println("IMU Heading: " + imu.getRotation2d().getDegrees());
  }



  /**
   * Gets the current position of the robot durring a simulation.
   * 
   * @return The current position of the robot as a Pose2d.
   */
  public Pigeon2SimState getSimulatedState() {
    return imu.getSimState();
  }


  /**
   * Inputs drive values into the swerve drive base.
   * 
   * @param driveXMetersPerSec the desired forward/backward interal motion, in meters per second.
   * @param driveYMetersPerSec the desired left/right lateral motion, in meters per second.
   * @param rotationRadPerSec the desired rotational motion, in radians per second.
   * @param isFieldOriented whether driving is field or robot-oriented.
   */
  public void drive(double driveXMetersPerSec, double driveYMetersPerSec, double rotationRadPerSec, boolean isFieldOriented) {
    if(omegaOverrideRadPerSec.isPresent()) {
      rotationRadPerSec = omegaOverrideRadPerSec.get();
    }

    if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      driveXMetersPerSec *= -1.0;
      driveYMetersPerSec *= -1.0;
    }

    if(driveXMetersPerSec != 0.0 || driveYMetersPerSec != 0.0 || rotationRadPerSec != 0.0) isLocked = false;

    if(isLocked) {
      setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI)),
        new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
        new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
        new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI))
      });
    }
    else {
      // Reduces the speed of the drive base for "Turtle" or "Sprint" modes.
      driveXMetersPerSec *= speedFactor;
      driveYMetersPerSec *= speedFactor;
      rotationRadPerSec *= speedFactor;

      // Represents the overall state of the drive base.
      ChassisSpeeds speeds =
        isFieldOriented
          ? ChassisSpeeds.fromFieldRelativeSpeeds(
            driveXMetersPerSec, driveYMetersPerSec, rotationRadPerSec, getHeading())
          : new ChassisSpeeds(driveXMetersPerSec, driveYMetersPerSec, rotationRadPerSec);

      ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

      // Uses kinematics to convert overall robot state to array of individual module states.
      final SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(discreteSpeeds);

      // Makes sure the wheels don't try to spin faster than the maximun speeds possible
      SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxModuleSpeedMetersPerSec);

      setModuleStates(states);
    }
  }

  public void runCharacterization(double volts) {
    frontLeftMod.runCharacterization(volts);
    frontRightMod.runCharacterization(volts);
    backLeftMod.runCharacterization(volts);
    backRightMod.runCharacterization(volts);
  }

  /**
   * Stops the driving of the drive base.
   * <p>Sets all drive inputs to zero. This will set the drive power of each module to zero while maintaining module headings.
   */
  public void stop() {
    drive(0.0, 0.0, 0.0, isFiledOriented);
  }

  public void Turns() {
    drive(0.0, 1.0, 0.5, isFiledOriented);
  }

  /**
   * Turns the modules to the X-lock position as long as drive inputs are zero.
   */
  public void lock() {
    isLocked = true;
  }

  /**
   * Sets the desired state for each swerve module.
   * <p>Uses PID and feedforward control (closed-loop) to control the linear and rotational values for the modules.
   * 
   * @param moduleStates An array module states to set. The order is FL, FR, BL, BR.
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    targetStatePublisher.set(moduleStates);
    frontLeftMod.setDesiredState(moduleStates[0], false);
    frontRightMod.setDesiredState(moduleStates[1], false);
    backLeftMod.setDesiredState(moduleStates[2], false);
    backRightMod.setDesiredState(moduleStates[3], false);
  }

  /** 
   * Returns the current motion of the drive base as a ChassisSpeeds.
   * 
   * @return A ChassisSpeeds representing the current motion of the drive base.
  */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Sets the ChassisSpeeds of the drive base.
   * 
   * @param chassisSpeeds the desired ChassisSpeeds.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(DriveConstants.kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public Translation2d getFieldRelativeVelocity() {
    return new Translation2d(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond).rotateBy(getHeading());
  }

  /**
   * Returns an array of module states of the drive base. The order is FL, FR, BL, BR.
   * 
   * @return An array of SwerveModuleState.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      new SwerveModuleState(frontLeftMod.getVelocityMetersPerSec(), frontLeftMod.getSteerEncAngle()),
      new SwerveModuleState(frontRightMod.getVelocityMetersPerSec(), frontRightMod.getSteerEncAngle()),
      new SwerveModuleState(backLeftMod.getVelocityMetersPerSec(), backLeftMod.getSteerEncAngle()),
      new SwerveModuleState(backRightMod.getVelocityMetersPerSec(), backRightMod.getSteerEncAngle())
    };
  }

  /**
   * Returns an array of CANcoder angles of the modules. The order is FL, FR, BL, BR.
   * 
   * @return An array of Rotation2d.
   */
  public Rotation2d[] getCanCoderAngles() {
    return new Rotation2d[] {
      frontLeftMod.getSteerEncAngle(),
      frontRightMod.getSteerEncAngle(),
      backLeftMod.getSteerEncAngle(),
      backRightMod.getSteerEncAngle()
    };
  }

  /**
   * Returns an array of module positions.
   * 
   * @return An array of SwerveModulePosition.
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeftMod.getPosition(),
      frontRightMod.getPosition(),
      backLeftMod.getPosition(),
      backRightMod.getPosition()
    };
  }

  /**
   * @return The current estimated position of the robot.
   */
  public Pose2d getPose() {
    return new Pose2d(
      poseEstimator.getEstimatedPosition().getX(),
      poseEstimator.getEstimatedPosition().getY(),
      poseEstimator.getEstimatedPosition().getRotation()
    );
  }

  /**
   * @return The current estimated pose of the robot, which will be mirrored if on the red alliance.
   * This is useful for checking the pose of the robot in an autonomous program, as PathPlanner paths
   * can mirror the blue side paths for use on the red side.
   */
  public Pose2d getBlueSidePose() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      return new Pose2d(16.4 - getPose().getX(), getPose().getY(), new Rotation2d(MathUtil.angleModulus(getPose().getRotation().getRadians() - Math.PI)));
    }
    else {
      return getPose();
    }
  }

  public void setHeading(Rotation2d heading) {
    imu.setYaw(MathUtil.inputModulus(heading.getDegrees(), 0.0, 360.0));
  }

  /**
   * Resets the current pose to (0, 0) with a heading of zero.
   */
  public void resetPose() {
    setPose(new Pose2d());
  }

  /**
   * Sets the pose of the robot.
   * 
   * @param pose The pose to set the robot to.
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(imu.getRotation2d(), getModulePositions(), pose);
  }

  public void setTranslation(Translation2d translation) {
    poseEstimator.resetPosition(imu.getRotation2d(), getModulePositions(), new Pose2d(translation, imu.getRotation2d()));
  }

  /**
   * Resets the measured distance driven for each module to zero.
   * <p>Resets the drive encoders of each module to zero.
   */
  public void resetDriveDistances() {
    frontLeftMod.resetDriveDistance();
    frontRightMod.resetDriveDistance();
    backLeftMod.resetDriveDistance();
    backRightMod.resetDriveDistance();
  }

  /**
   * Returns the average distance of each module to get an overall distance driven by the robot.
   * 
   * @retrun The overall distance driven by the robot in meters.
   */
  public double getAverageDriveDistanceMeters() {
    return (
      (frontLeftMod.getDriveDistanceMeters() +
      frontRightMod.getDriveDistanceMeters() +
      backLeftMod.getDriveDistanceMeters() +
      backRightMod.getDriveDistanceMeters()) 
      / 4.0
    );
  }

  /**
   * Returns the average velocity of each module to get an overall velocity of the robot.
   * 
   * @return The overall velocity of the robot in meters per second.
   */
  public double getAverageVelocityMetersPerSec() {
    return (
      (frontLeftMod.getVelocityMetersPerSec() +
      frontRightMod.getVelocityMetersPerSec() +
      backLeftMod.getVelocityMetersPerSec() +
      backRightMod.getVelocityMetersPerSec()) 
      / 4.0
    );
  }

  public double getAverageDriveVoltage() {
    return (
      (frontLeftMod.getDriveVoltage() +
      frontRightMod.getDriveVoltage() +
      backLeftMod.getDriveVoltage() +
      backRightMod.getDriveVoltage()) 
      / 4.0
    );
  }

  /**
   * Returns the average direction of each module to get an overall direction of travel of the robot.
   * 
   * @return The overall direction of travel of the robot.
   */
  public Rotation2d getDirectionOfTravel() {
    return new Rotation2d(
      ((frontLeftMod.getSteerEncAngle().plus(new Rotation2d(frontLeftMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0))).getRadians() +
      (frontRightMod.getSteerEncAngle().plus(new Rotation2d(frontRightMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0))).getRadians() +
      (backLeftMod.getSteerEncAngle().plus(new Rotation2d(backLeftMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0))).getRadians() +
      (backRightMod.getSteerEncAngle().plus(new Rotation2d(backRightMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0))).getRadians())
    / 4.0
    );
  }

  /**
   * Returns the average velocity in the direction relative to the robot.
   * 
   * @param relativeHeading The relative heading of the robot, where zero is the front of the robot.
   * 
   * @return the velocity in the direction relative to the robot in meters per second.
   */
  public double getRelativeVelocityMetersPerSec(Rotation2d relativeHeading) {
    return getDirectionOfTravel().minus(relativeHeading).getRadians() * getAverageVelocityMetersPerSec();
  }

  /**
   * Returns the current heading of the robot from the gyro.
   * 
   * @return The current heading of the robot as a Rotation2d.
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Returns the current pitch of the robot from the gyro.
   * 
   * @return The current pitch of the robot as a Rotation2d.
   */
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(imu.getPitch().getValueAsDouble());
  }

  /**
   * Returns the current roll of the robot from the gyro.
   * 
   * @return The current roll of the robot as a Rotation2d.
   */
  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(imu.getRoll().getValueAsDouble());
  }

  /**
   * Sets the gyro heading to zero.
   */
  public void resetHeading() {
    poseEstimator.resetPosition(
      imu.getRotation2d(),
      getModulePositions(),
      new Pose2d(
        getPose().getTranslation(),
        DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.fromDegrees(180.0) : Rotation2d.fromDegrees(0.0)
      )
    );
  }
}
