// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software released under the WPILib BSD licence.


  //-------------------------------------
  //         Importing Software and Code, defining CAN IDs
  //-------------------------------------

package frc.robot;


import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class Robot extends TimedRobot {


 // CAN IDs
 private final WPI_VictorSPX leftMaster   = new WPI_VictorSPX(11);
 private final WPI_VictorSPX rightMaster  = new WPI_VictorSPX(12);
 private final WPI_VictorSPX leftFollower = new WPI_VictorSPX(13);
 private final WPI_VictorSPX rightFollower= new WPI_VictorSPX(14);
 private final WPI_TalonSRX pickupMotor    = new WPI_TalonSRX(22);
 private final Pigeon2 pidgey     = new   Pigeon2(6);

 private DifferentialDrive drive;
 private final XboxController driver = new XboxController(0);


 // ---------------------------------------------------------------------------
 // One-time setup
 // ---------------------------------------------------------------------------
 @Override
 public void robotInit() {

   // m_field.getObject("traj").setTrajectory(m_trajectory);
   System.out.println(pidgey.isConnected());
   pidgey.setYaw(0.0);



   // Reset all SPX settings
   leftMaster.configFactoryDefault();
   rightMaster.configFactoryDefault();
   leftFollower.configFactoryDefault();
   rightFollower.configFactoryDefault();
   pickupMotor.configFactoryDefault();


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
 //these are variables used in the teleop method
 static double mainspeed = 0.75;
 static double PickupSpeed;
 static double revertAngle = 0;
  @Override
  public void teleopPeriodic() {
   

    //-------------------------------------
    //         Speed and setting driving motors
    //-------------------------------------
    if (driver.getLeftBumperButton()){
      //lower mainspeed for lowerspeed mode on push
      mainspeed = 0.5;
    }
    
    if (driver.getRightBumperButton()) {
      //speed back to normal mode on push
      mainspeed = 0.75;
    }
    
    // Split-arcade driving: left-stick Y for throttle, right-stick X for turn
    drive.arcadeDrive(
        (-driver.getLeftY() * mainspeed /* modifying raw joysticl input by mainspeed set above */),   // invert so forward is positive
        (-driver.getRightX() * mainspeed)); // invert so right is positive
        
    double LeftYJoy = driver.getLeftY();
    double RightXJoy = driver.getRightX();

 
 
    if (driver.getYButton()) { //unused if y button is pressed
     

    }
    
    if (driver.getAButton()) { //unused if a button is pressed
      
             
    }
 
 

    if (driver.getXButton()) {
      
     
    }
    
    if (driver.getBButton()) {
    
    }
 

    //-------------------------------------
    //         Dispenser System
    //-------------------------------------

    if (driver.getRightTriggerAxis() > 0.1 || driver.getLeftTriggerAxis() > 0.1) //if either trigger is pulled at least a bit
      {

        if (driver.getRightTriggerAxis() > 0.3) { //if right trigger pulled, then dispenser at that speed ("right one dispenses" -adlai)
          PickupSpeed = driver.getRightTriggerAxis(); 
          // System.out.println("Aux speed: " + PickupSpeed);
      }else if (driver.getLeftTriggerAxis() > 0.3){ //if left trigger pulled, then run collecter at that speed
        PickupSpeed = driver.getLeftTriggerAxis() * 1.0 * -1;
      }
      
    } else{
      PickupSpeed = 0.0; //if no triggers, dont run it
     }

    pickupMotor.set(PickupSpeed);//sets pickup motor speed as set above


    //-------------------------------------
    //         Pidgey
    //-------------------------------------
     if (driver.getStartButtonPressed()==true){ //if start button is pressed, put pidgey yaw to 0, untested and not used at time of writing
      revertAngle = pidgey.getYaw().getValueAsDouble();
      pidgey.setYaw(0.0);
      System.out.println("### Yaw Reset! ###");
     }


      
   System.out.println(
      "POV: " + driver.getPOV() + 
      " \n| AuxSpeed: " + PickupSpeed +
      " \n| LJoy: " + Math.round(LeftYJoy*1000)/1000 + 
      " \n| Mainspeed: " + Math.round(mainspeed*1000)/1000 + 
      " \n| RJoy: " + Math.round(RightXJoy*1000)/1000 + 
      " \n| Drive Powah: " + Math.round(mainspeed*1000)/1000 +
      
      " \n| Angle yaw : " + Math.round(pidgey.getYaw().getValueAsDouble()) +
      " \n| Angle pitch : " + Math.round(pidgey.getPitch().getValueAsDouble()) +
      " \n| Angle roll : " + Math.round(pidgey.getRoll().getValueAsDouble()));
 }
    
 
 
   
    //dpad here
    //if (driver.getdpad)
   
 
 
    /*
    dpad here
    if (driver.getdpad)
    */
  
  //-----------------------------------------------------------------------------------------------------------------
  // Potential new system of calling functions below.
  // Put inside @overide below driving systems and replace other if statements with the ones below,
  // make sure static void statements are outside of the @override statement.
  //-----------------------------------------------------------------------------------------------------------------
  // Run the aux motor while RB is held


}
