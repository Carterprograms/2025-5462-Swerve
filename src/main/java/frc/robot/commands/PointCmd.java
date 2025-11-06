package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSys;
import frc.robot.util.limelight.LimelightHelpers;

public class PointCmd extends Command{
    private final SwerveSys swerveSys;
    private final double kP = 0.02; // Proportional constant for centering
    private final double tolerance = 1.0; // Degress of error tolerance

    public PointCmd(SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
        addRequirements(swerveSys);
    }
    
    @Override
    public void execute() {
        // Get the horizontal offset (tx) from the limelight
        double tx = LimelightHelpers.getTX("limelight");

        // Calculate the rotational speed to center the robot
        double rotationSpeed = kP * tx;

        // Limit the rotation speed to avoid excessive movement
        rotationSpeed = Math.max(-0.5, Math.min(0.5, rotationSpeed));

        // Command the swerve drive to rotate
        swerveSys.drive(0, 0, rotationSpeed, false);
    }

    @Override
    public boolean isFinished() {
        // Check if the target is within the tolerance
        return Math.abs(LimelightHelpers.getTX("limelight")) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        swerveSys.drive(0, 0, 0, false);
    }
}
