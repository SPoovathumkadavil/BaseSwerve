package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Set all valued before passing into Swerve Module
 */
public class ModuleConfig {

    public double angleP = 0.0020645;
    public double angleI = 0.00001;
    public double angleD = 0;

    public double driveP = 0.0020645; // TODO: Check
    public double driveI = 0;
    public double driveD = 0;

    public int driveId;
    public int angleId;

    public int driveCurrentLimit = 10;
    public int angleCurrentLimit = 20;

    public Translation2d location;

    public boolean driveInverted;
    public boolean angleInverted;
}
