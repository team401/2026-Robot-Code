package frc.robot.constants;

public class CANBusAssignment {

  // Drivetrain CAN IDs

  public final Integer frontLeftDriveKrakenId = 7;
  public final Integer frontLeftSteerKrakenId = 2;
  public final Integer frontLeftEncoderId = 9;

  public final Integer frontRightDriveKrakenId = 1;
  public final Integer frontRightSteerKrakenId = 6;
  public final Integer frontRightEncoderId = 12;

  public final Integer backLeftDriveKrakenId = 5;
  public final Integer backLeftSteerKrakenId = 8;
  public final Integer backLeftEncoderId = 10;

  public final Integer backRightDriveKrakenId = 3;
  public final Integer backRightSteerKrakenId = 4;
  public final Integer backRightEncoderId = 11;

  public final Integer kPigeonId = 13;

  // The rest of the CAN IDs Should start from 14 and go up from there

  public final Integer turretKrakenId = 14;

  public final Integer indexerKrakenId = 15;

  public final Integer hopperKrakenId = 16;
  public final Integer hoodKrakenId = 17;

  public final Integer homingSwitchCANdiID = 18;

  public final Integer intakePivotMotorId = 19;
  public final Integer intakeRollersLeadMotorId = 20;
  public final Integer intakeRollersFollowerMotorId = 21;

  public final Integer shooterLeaderId = 22;
  public final Integer shooterFollowerId = 23;

  public final Integer climberKrakenId = 24; // TODO: Verify this ID
}
