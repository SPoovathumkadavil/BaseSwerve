package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.ModuleConfig;

public class Constants {

    public class Swerve {

        public static final double DRIVE_RATIO = 0.0472867872; // TODO: CHECK
        public static final double ANGLE_RATIO = 16.8;
        public static final double WHEEL_DIAMETER_METERS = 0.0762;

        public static final double MAX_LINEAR_SPEED_METERS_PER_SECOND = 4; // TODO: CHANGE

        // CONFIGS
        public static ModuleConfig flConfig = new ModuleConfig();
        public static ModuleConfig frConfig = new ModuleConfig();
        public static ModuleConfig blConfig = new ModuleConfig();
        public static ModuleConfig brConfig = new ModuleConfig();

        static {
            flConfig.driveId = 1;
            flConfig.angleId = 2;
            frConfig.driveId = 4;
            frConfig.angleId = 5;
            blConfig.driveId = 7;
            blConfig.angleId = 8;
            brConfig.driveId = 10;
            brConfig.angleId = 11;

            // TODO: Should Use CANCODER?

            flConfig.location = new Translation2d(14.25, 14.25);
            frConfig.location = new Translation2d(14.25, -14.25);
            blConfig.location = new Translation2d(-14.25, 14.25);
            brConfig.location = new Translation2d(-14.25, -14.25);
            
            flConfig.driveInverted = true;
            flConfig.angleInverted = true;
            frConfig.driveInverted = false;
            frConfig.angleInverted = true;
            blConfig.driveInverted = true;
            blConfig.angleInverted = true;
            brConfig.driveInverted = false;
            brConfig.angleInverted = true;
        }
        
    }
    
}
