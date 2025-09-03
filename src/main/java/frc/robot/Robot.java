// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software released under the WPILib BSD licence.
//  an edits 
//anotther edit
package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;


public class Robot extends TimedRobot {


 // CAN IDs
 private final WPI_VictorSPX leftMaster   = new WPI_VictorSPX(11);
 private final WPI_VictorSPX rightMaster  = new WPI_VictorSPX(12);
 private final WPI_VictorSPX leftFollower = new WPI_VictorSPX(13);
 private final WPI_VictorSPX rightFollower= new WPI_VictorSPX(14);
 private final WPI_TalonSRX rightAux     = new WPI_TalonSRX(22);
 private final WPI_TalonSRX leftAux     = new WPI_TalonSRX(21);

 private DifferentialDrive drive;
 private final XboxController driver = new XboxController(0);

 private final Field2d m_field = new Field2d();
 private Pose2d m_pose = new Pose2d();
 private Timer timer = new Timer();
 // ---------------------------------------------------------------------------
 // One-time setup
 // ---------------------------------------------------------------------------
 @Override
 public void robotInit() {
    
    /*Trajectory m_trajectory = 
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)), 
          List.of(new Translation2d(1,1), new Translation2d(2, -1)), 
          new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
*/
    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(m_pose);

   // m_field.getObject("traj").setTrajectory(m_trajectory);

   // Reset all SPX settings
   leftMaster.configFactoryDefault();
   rightMaster.configFactoryDefault();
   leftFollower.configFactoryDefault();
   rightFollower.configFactoryDefault();
   rightAux.configFactoryDefault();
   leftAux.configFactoryDefault();


   // Followers
   leftFollower.follow(leftMaster);
   rightFollower.follow(rightMaster);
   leftFollower.setInverted(InvertType.FollowMaster);
   rightFollower.setInverted(InvertType.FollowMaster);


   // Drive-side inversion (adjust if your gearbox is mirrored)
   leftMaster.setInverted(false);
   rightMaster.setInverted(true);

   // DifferentialDrive wrapper
   drive = new DifferentialDrive(leftMaster, rightMaster);
   SendableRegistry.addChild(drive, leftMaster);
   SendableRegistry.addChild(drive, rightMaster);
 }


       




 // ---------------------------------------------------------------------------
 // Tele-op loop (called every 20 ms by default)
 // ---------------------------------------------------------------------------


  // ---------------------------------------------------------------------------
 // Current system
 // ---------------------------------------------------------------------------

 //vars
 static double mainspeed = 0.75;
 static double LAuxSpeed;
 static double RAuxSpeed;
 static double turningSpeed = 1;
 static double driveSpeed = 0;

  @Override
  public void teleopPeriodic() {
   
    if (driver.getLeftBumperButton()){
      //LB();
      mainspeed = 0.5;
      turningSpeed = 0.65;
    } else {
      mainspeed = 0.7;
      turningSpeed = 0.85;
    }
    if (driver.getRightBumperButton()) {
      //RB();
      mainspeed = 1;
      turningSpeed = 1;
    } else {
      mainspeed = 0.7;
      turningSpeed = 0.85;
    }
    





    // Split-arcade driving: Left-stick Y for throttle, Left-stick X for turn
    
    // Changes
    // . Turning is too fast in slow mode - Fixed with parabolic curve in throttle.
    // . Turning wile driving is too slow in slow mode
    // . Make steering exponential
    
    double LeftYJoy = -driver.getLeftY();
    double RightXJoy = -driver.getRightX();
    
    double turnSpeed = RightXJoy * turningSpeed;
    if (LeftYJoy < 0.05 && LeftYJoy > -0.05) {
      driveSpeed = 0;
    }
    else if (LeftYJoy > 0.05) {
      // driveSpeed = (LeftYJoy * LeftYJoy * mainspeed + 0.1) * (LeftYJoy * LeftYJoy * mainspeed + 0.1);
      driveSpeed = (LeftYJoy * LeftYJoy * mainspeed + 0.1);
    } 
    else if (LeftYJoy < 0.05) {
      // driveSpeed = (LeftYJoy * LeftYJoy * mainspeed + 0.1) * (LeftYJoy * LeftYJoy * mainspeed + 0.1);
      driveSpeed = -(LeftYJoy * LeftYJoy * mainspeed + 0.1);
    } 
    else {
      driveSpeed = -0.0;
    }
    drive.arcadeDrive(
        (driveSpeed),   // invert so forward is positive
        (turnSpeed)); // invert so right is positive
        
    //runAux();

 
 
    if (driver.getYButton()) {
     //Y();
     System.out.println("Y pressed");
    }
    
    if (driver.getAButton()) {
      //A();
      System.out.println("A pressed");         
    }
 
 
    // Run console log of button pressed while x is held
    if (driver.getXButton()) {
      //X();
      System.out.println("X pressed");
      }
    
     if (driver.getBButton()) {
       //B();
       System.out.println("B pressed");
     }
 
      
    if (driver.getPOV() == -1){
      RAuxSpeed = 0;
    } 
    else{
      if (driver.getPOV() == 0){
        RAuxSpeed = 0.01;
        try{
          Thread.sleep(1000);
        } catch(InterruptedException e){
          System.out.println("Interupted");
        }
        RAuxSpeed = 0.0;
      }
      if (driver.getPOV() == 180){
        RAuxSpeed = -0.01;
      }

      System.out.println(RAuxSpeed);
    }
    leftAux.set(RAuxSpeed);
    System.out.println(driver.getPOV());
    System.out.println(
      "POV: " + driver.getPOV() + 
      " | RAuxS: " + RAuxSpeed + 
      " | LJoy: " + LeftYJoy + 
      " | Mainspeed: " + mainspeed + 
      " || RJoy: " + RightXJoy + 
      " | TurnSpeed: " + turnSpeed + 
      " | Drive Powah: " + driveSpeed);

      if (driver.getRightTriggerAxis() > 0.1 || driver.getLeftTriggerAxis() > 0.1) 
      {
      //RT();

        if (driver.getRightTriggerAxis() > 0.3) {
      LAuxSpeed = driver.getRightTriggerAxis();
      // System.out.println("Aux speed: " + LAuxSpeed);
     }else if (driver.getLeftTriggerAxis() > 0.3){
      LAuxSpeed = driver.getLeftTriggerAxis() * 0.8 * -1;

     }
      
     
    } else{
      LAuxSpeed = 0.2;
     }
     System.out.println("Aux speed: " + LAuxSpeed);
     leftAux.set(LAuxSpeed);

 }
//autonomous
 @Override
 public void autonomousInit(){
  timer.reset();
  timer.start();
 }
 @Override
 public void autonomousPeriodic() {
 /* 
  drive.arcadeDrive(
         (1.0),  
         (0.0));
*/
   if(timer.get() <= 10.0){
   drive.arcadeDrive(
         (0.7),  
         (0.0));
  }
 else{
  drive.arcadeDrive(
         (0.0), 
         (0.0));
  leftAux.set(-0.3); 
 }

}
}