package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.I2C.Port;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); 
        public static final double kDriveMotorGearRatio = 1/6.75;
        public static final double kTurningMotorGearRatio = 1/12.8;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(22.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 10;  // Double check motor can IDs
        public static final int kBackLeftDriveMotorPort = 3;     // Consider abbreviating these to kFLDriveMotorID
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 5;

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 9;
        public static final int kFrontRightTurningMotorPort = 2;
        public static final int kBackRightTurningMotorPort = 6;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        

        //public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        //public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        //public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        //public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        //public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -0.75;
        //public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.49 ;


        //public static final int kFrontLeftDriveAbsoluteEncoderPort = 0; //NEED
       //public static final int kBackLeftDriveAbsoluteEncoderPort = 3;//maybe switch with 3
        //public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        //public static final int kBackRightDriveAbsoluteEncoderPort = 2;

        public static final class AbsoluteEncoders
        {
            public static final class Front
            {
                public static final class Left
                {
                    public static final double Offset = 1.63;  //NEED        
                    public static final boolean Reversed = true ;
                    public static final int Port = 0;
                }// 3.14 1.57, 0.785
                public static final class Right
                {
                    public static final double Offset = -0.12;  //NEED       
                    public static final boolean Reversed = true ;
                    public static final int Port = 1;
                }
            }

            public static final class Back
            {
                public static final class Left
                {
                    public static final double Offset = 2.9;  //NEED        
                    public static final boolean Reversed = true ;
                    public static final int Port = 3;
                }
                public static final class Right
                {
                    public static final double Offset = -0.69;  //NEED       
                    public static final boolean Reversed = true ;
                    public static final int Port = 2;
                }

            }
        }
      
        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.6;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.2;
        public static final double kPYController = 0.2;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;
    }

    public static final class NoteConstants{
        public static final int IntakeMotorID = 8;
        public static final int SharedMotorID = 11;
    }
}
