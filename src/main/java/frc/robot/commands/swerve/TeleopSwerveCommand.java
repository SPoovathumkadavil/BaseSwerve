package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TeleopSwerveCommand extends Command {

    private CommandXboxController xbox;
    private SwerveDrive swerve;

    private DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

    public static final double MAX_FORWARD_SENSITIVITY = 4;
    public static final double MAX_SIDEWAYS_SENSITIVITY = 4;
    public static final double MAX_ROTATIONAL_SENSITIVITY = 3.5;
    public static final double MIN_SENSITIVITY = 0.2;

    public TeleopSwerveCommand(CommandXboxController xbox) {
        this.xbox = xbox;
        swerve = SwerveDrive.getInstance();

        addRequirements(swerve);
    }

    @Override
    public void execute() {

        double sensMod = Math.max(1 - xbox.getLeftTriggerAxis(), MIN_SENSITIVITY);
        double forward = -xbox.getLeftY() * MAX_FORWARD_SENSITIVITY * sensMod;
        double left = -xbox.getLeftX() * MAX_SIDEWAYS_SENSITIVITY * sensMod;
        double rotational = -xbox.getRightX() * MAX_ROTATIONAL_SENSITIVITY * sensMod;

        switch (driveMode) {
            case ROBOT_CENTRIC:
                swerve.driveRobotCentric(new ChassisSpeeds(
                    forward,
                    left,
                    rotational
                ));
                break;
        }

    }
    
    enum DriveMode {
        ROBOT_CENTRIC
    }
}
