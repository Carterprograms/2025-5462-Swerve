package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

public class Constants {


    public static final class CANDevices {
        // Set these CAN ID values to the those of your robot, or change your CAN ID's to match this convention.

        public static final int imuId = 9;

        public static final int frontLeftCanCoderId = 11;
        public static final int frontLeftSteerMtrId = 3;
        public static final int frontLeftDriveMtrId = 4;

        public static final int frontRightCanCoderId = 10;
        public static final int frontRightSteerMtrId = 5;
        public static final int frontRightDriveMtrId = 6;

        public static final int backLeftCanCoderId = 12;
        public static final int backLeftSteerMtrId = 1;
        public static final int backLeftDriveMtrId = 2;

        public static final int backRightCanCoderId = 13;
        public static final int backRightSteerMtrId = 7;
        public static final int backRightDriveMtrId = 8;

        public static final int m_rightLiftMtrId = 14;
        public static final int m_leftLiftMtrId = 15;

        public static final int m_conveyorMtrId = 32;

        //Leader Mtr is Right Mtr and Follower Mtr is Left Mtr
        public static final int leaderRollerMtrId = 21;
        public static final int followerRollerMtrId = 22;

        //Leader Mtr is Right Mtr and Follower Mtr is Left Mtr
        public static final int rightPivotMtrId = 23;
        public static final int leftPivotMtrId = 24;

        //Leader Mtr is Left Mtr and Follower Mtr is Right Mtr
        public static final int rightClimberMtrId = 25;
        public static final int leftClimberMtrId = 26;

        public static final int rightIntakeOutMtrId = 30;
        public static final int leftIntakeOutMtrId = 31;

        public static final int spacebarMtrId = 27;
    }

    public static final class ControllerConstants {

        public static final int driverGamepadPort = 0;

        public static final int operatorGamepadPort = 1;

        public static final double joystickDeadband = 0.15;

        public static final double triggerPressedThreshhold = 0.25;

        public static final int driverRightJoystick = 1;

        public static final int buttonPanelPort = 2;
    }
    
    public static final class DriveConstants {
        /**
         * The track width from wheel center to wheel center.
         */
        // Make sure to measure from the center of each wheel
        public static final double trackWidth = Units.inchesToMeters(19.6875);

        /**
         * The track length from wheel center to wheel center.
         */
        // mature sure to measure from the center of each wheel
        public static final double wheelBase = Units.inchesToMeters(19.6875);

        /**
         * The SwerveDriveKinematics used for control and odometry.
         */
        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0),  // front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // back left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // back right
            );

        /**
         * The gear reduction from the drive motor to the wheel.
         * 
         * The drive gear ratios for the different levels can be found from the chart at
         * 
         */
        // This is the gear ratio for L3 modules.
        public static final double driveMtrGearReduction = (16.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);

        /**
         * The gear reduction from the steer motor to the wheel.
         */
        public static final double steerMtrGearReduction = 150/7;

        public static final double wheelRadiusMeters = Units.inchesToMeters(2);
        public static final double wheelCircumferenceMeters = 2.0 * wheelRadiusMeters * Math.PI;

        public static final double driveMetersPerEncRev = wheelCircumferenceMeters * driveMtrGearReduction;
        public static final double driveMetersPerSecPerMtrRPM = driveMetersPerEncRev / 60.0;

        public static final double steerRadiansPerEncRev = 2 * Math.PI * steerMtrGearReduction;

        public static final double freeMetersPerSecond = 6784 * driveMetersPerSecPerMtrRPM;

        /**
         * The maximum possible speed a module can be driven. Used for desaturation.
         */
        public static final double maxModuleSpeedMetersPerSec = 10;

        public static final double maxDriveSpeedMetersPerSec = 10;

        /**
         * The rate the robot will spin with full Rot command.
         */
        public static final double maxTurnSpeedRadPerSec = 3.0 * Math.PI;

        // Set line up the swerve modules and set these values.

        // The bolt heads should be pointing to the right. These values are subtracted from the CANCoder reading,
        // so they should be the raw CANCoder value when set straight. These values should be between 0 and 360
        // degrees.
        public static final Rotation2d frontLeftModOffset = Rotation2d.fromDegrees(124.09); // 122.43, 318.164, 135.97
        public static final Rotation2d frontRightModOffset = Rotation2d.fromDegrees(117.9); // 184.12, 234.756, 134.5
        public static final Rotation2d backLeftModOffset = Rotation2d.fromDegrees(-150.01); // 62, 252.15804, -164.5
        public static final Rotation2d backRightModOffset = Rotation2d.fromDegrees(10.36); // 82.7, 252.15804, 40.07

        // You may want to change this value.
        public static final int driveCurrentLimitAmps = 70;
        public static final double brownoutVoltage = 6.25;
        
        // These values should be fine, but if the modules start to rattle you may want to play with the steer PID values.
        public static final double drivekP = 0.13;//0.005;
        public static final double drivekD = 0.0;

        public static final double steerkP = 0.37431;
        public static final double steerkD = 0.27186;

        public static final double ksVolts = 0.667;
        public static final double kvVoltSecsPerMeter = 1.78; //0.3622 // 1.714
        public static final double kaVoltSecsPerMeterSq = 0.55;

        public static final double kvVoltsSecsPerRot = (0.1016 * Math.PI) * kvVoltSecsPerMeter;
        public static final double kaVoltSecsPerRotSq = (0.1016 * Math.PI) * kaVoltSecsPerMeterSq;

        public static final double kvVoltsSecsPerRad = kvVoltsSecsPerRot / (2 * Math.PI);
        public static final double kaVoltSecsPerRadSq = kaVoltSecsPerRotSq / (2 * Math.PI);

        public static final SimpleMotorFeedforward driveFF =
            new SimpleMotorFeedforward(ksVolts, kvVoltSecsPerMeter, kaVoltSecsPerMeterSq);
    }

    public static final class AutoConstants {

        // These drive and rotation PID constants most likely need to be tuned for better accuracy.
        public static final double drivekP = 2.0 * 4.0; // 12.8
        public static final double drivekD = 0.02 * 4.0; //0.0625; // .085

        public static final double rotkP = 5.5; // 1.27
        public static final double rotkD = 0.5; // 0.5

        // Auto aim PID values should ideally be the same as the PathPlanner rotation ones. They are separate for safe measure.

        public static final double autoAimkP = 10.9;
        public static final double autoAimkD = 0.5;

        public static final double autoLiftkP = 0;
        public static final double autoLiftkD = 0;

        public static final double autoAimToleranceDeg = 0.2;

        public static final double autoAimTurnSpeedRadPerSec = 2.0 * Math.PI;
        public static final double autoAumTurnAccelRadPerSecSq = 3.0 * Math.PI;

        public static final Pose2d driveToAmpWaypoint = new Pose2d(1.83, 7.81, Rotation2d.fromDegrees(-90.0));
        public static final Pose2d driveToAmpTargetPoint = new Pose2d(1.83, 7.61, Rotation2d.fromDegrees(-90.0));

        public static final double driveToAmpMaxVelMetersPerSec = 4.0;
        public static final double driveToAmpMaxAccelMetersPerSecSq = 3.0;

        public static final double liftMaxVelMetersPerSec = 0.2;
        public static final double liftMaxAccelMeterPerSecSq = 0.1;

        public static final double subwooferShotThreshold = 1.8;

        public static final double offsetSubwooferShotThreshold = 1.0;

    }

    public class RollerConstants {
    
        public static final int maxRollerCurrentAmps = 55;

        public static final double gearRatio = 0.8;

        public static final double freeSpeedRPM = 6784.0;

        public static final double maxRPM = freeSpeedRPM / gearRatio;

        public static final double feedForward = 0.00018;

        public static final double kP = 0.0002; // 0.0002

        public static final double kD = 0.0025; // 0.003
        
        public static final double fireRPM = 5250.0;
        
        public static final double ampRPM = 900.0;
        
        public static final double intakeRPM = 6200.0;

        public static final double rollerDiameterMeters = Units.inchesToMeters(2.0);

        public static final double rollerCircumferenceMeters = rollerDiameterMeters * Math.PI;

        public static final double metersPerSecondPerRPM = rollerCircumferenceMeters / 60.0;

        public static final int sensorHasNoteADCThreshold = 200;

        public static final double sensorTimeIntervalSecs = 0.4;
    }

    public class PivotConstants {

        public static final int maxPivotCurrentAmps = 50;

        public static final double gearRatio = 45.0;

        public static final double kP = 0.012; // 0.035;
        public static final double kD = 0.00025; // 0.00037;

        public static final double degPerEncRev = 360.0 / gearRatio;
        public static final double degPerSecPerRPM = 360.0 / (60.0 * gearRatio);

        public static final double freeSpeedRPM = 6784.0 / gearRatio;

        public static final double maxVelDegPerSec = 800.0; // 400.0;

        public static final double maxAccelDegPerSecSq = 575.0; // 575.0;

        public static final double maxManualDegPerSec = 180.0;

        public static final double maxManualDegPerSecSq = 375.0;

        public static final double trapPresetDeg = 150.0;

        public static final double ampPresetDeg = 67.0;
        
        public static final double sourcePresetDeg = 68.0;

        public static final double groundPresetDeg = 183.0;

        public static final double homePresetDeg = 0.0;

        public static final double podiumPresetDeg = 82.4;

        public static final float lowerLimitDeg = 0f;

        public static final float upperLimitDeg = 181f;

        public static final double podiumCorrectionIncrementDeg = .01;

        public static final double toleranceDeg = 0.5;

        public static final double absPivotEncOffsetDeg = 208.0 - 60.0;

        /*
         Used to calculate the approximate time of flight of the note.
         */
        public static final double pivotHeightMeters = Units.inchesToMeters(10.5);

        /*
         Takes distance to speaker in meters as the key and pivot angle in degrees as the value.
         */
        public static final InterpolatingDoubleTreeMap pivotDegSpeakerShotInterpolator = constructPivotInterpolator();
        
        private static InterpolatingDoubleTreeMap constructPivotInterpolator() {
            InterpolatingDoubleTreeMap pivotInterpolator = new InterpolatingDoubleTreeMap();

            // data points with coordinate (lateral distance to speaker [meters], pivot angle [degrees])
            pivotInterpolator.put(1.23, 59.0 - 2.0);
            pivotInterpolator.put(2.24, 79.7 - 2.0);
            pivotInterpolator.put(2.77, 84.0 - 3.0);
            pivotInterpolator.put(3.12, podiumPresetDeg);
            // pivotInterpolator.put(3.5, 77.0);

            return pivotInterpolator;
        }
    }

    public class ClimberConstants {
        public static final double gearReduction = 20.0;

        public static final float climberForwardLimit = 147.0f;
        public static final float climberReverseLimit = 0.1f;

        public static final double climberSpeedFactor = 1.0;
    }

    public class SpacebarConstants {
        public static final double gearReduction = 25.0;

        public static final float spacebarForwardLimit = 5.0f;
        public static final float spacebarReverseLimit = 0.01f;

        public static final double kP = 0.015;
        public static final double kD = 0.00025;
        public static final double maxVelDegPerSec = 2000.0;
        public static final double maxAccelDegPerSecSq = 1500.0;

        public static final double spacebarHomeDeg = -2.0;
        public static final double spacebarOutDeg = 200.0;
    }

    public class LightsConstants {
        public static final Color blueAllianceColor = new Color(0, 0, 255);
        public static final Color redAllianceColor = new Color(255, 0, 0);
        public static final Color noAllianceColor = new Color(200, 255, 200);

        public static final Color hasNoteColor = new Color(0, 255, 0);

        public static final double partyModeHueIncrement = 5;
        public static final double partyModeTranslationTimeSec = 0.05;

        public static final double brightnessPercentage = 0.5;
    }

    public class VisionConstants {
        public static final String frontLimelightName = "limelight";
        public static final String backLimelightName = "limelight";

        public static final double targetAreaPercentThreshold = 0.15;
    }

    public class FieldConstants {
        public static final Translation2d blueAllianceSpeakerPose = new Translation2d(0.0, 5.55);
        public static final Translation2d redAllianceSpeakerPose = new Translation2d(16.54, 5.55);

        public static final Translation2d blueReefPose = new Translation2d(4.5, 4);

        public static final double speakerTargetHeightMeters = 0.0762;
    }
}