// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;

import edu.wpi.first.wpilibj.TimedRobot; //include WPILib libraries
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.POVButton;
import frc.robot.subsystems.DriveTrain;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  Joystick driveStick = new Joystick(0); // joystick setup
  Joystick buttonPad = new Joystick(1);

  POVButton povButtonUp = new POVButton(driveStick, 0); // setup POV buttons. "0" is the angle position as if on a
                                                        // circle
  POVButton povButtonDown = new POVButton(driveStick, 180);
  POVButton povButtonLeft = new POVButton(driveStick, 270);
  POVButton povButtonRight = new POVButton(driveStick, 90);

  DriveTrain drive = new DriveTrain();

  Spark cpSpinner = new Spark(1);             //setup spark/sparkMax controllers 
  PWMSparkMax winch = new PWMSparkMax(2);
  PWMSparkMax flyWheel = new PWMSparkMax(3);
  Spark intakeRollers = new Spark(4);
  Spark intakeArm = new Spark(5);
  Spark telescope = new Spark(6);
  Spark carousel = new Spark(7);
  Spark shooterIntake = new Spark(8);
  Spark blinkin = new Spark(9);   //setup Blinkin LED Strip controller. Yes it uses the spark class.
                                  // see https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf for documentation

  Compressor compressor;   //setup pnuematics
  DoubleSolenoid shooterIntakeActuator = new DoubleSolenoid(0,1);
  DoubleSolenoid cpSpinnerActuator = new DoubleSolenoid(2,3);

  Timer shootDelayTimer = new Timer(); //setup various timers

  boolean ballsHaveBeenShot = false;
  
  double lastTimeStamp = 0;
  double xLastError = 0;
  double yLastError = 0;
  
  int buttonIsHeld = 0;
  
  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
      shootDelayTimer.reset();
      lastTimeStamp = Timer.getFPGATimestamp();
      xLastError = 0;
      yLastError = 0;
    }
  @Override
  public void autonomousPeriodic() {
    
    /*if(shootDelayTimer.get()< 2) {
      flDrive.set(ControlMode.PercentOutput, 0.5);
      frDrive.set(ControlMode.PercentOutput, 0.5);
      blDrive.set(ControlMode.PercentOutput, 0.5);
      brDrive.set(ControlMode.PercentOutput, 0.5);   
    }*/
    
    
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //set LED's to forced on
    // The angle offset to the target on the horizontal axis.
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    // The angle offset to the target on the vertical axis.
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    
    if (-1 >= tx || tx >= 1 || -1 >=ty || ty >= 1 ){
    
      if (ballsHaveBeenShot == false) {
    
        //variable setup
        // Constant for P
        double xKP = -0.1;
        // Constant for I
        double xKI = 1;
        // Constant for D
        double xKD = 0.05;
        // The point you want to get the robot to.
        double xSetpoint = 0;
        double xILimit = 2;
        
        // Constant for P
        double yKP = -0.1;
        // Constant for I
        double yKI = 1;
        // Constant for D
        double yKD = 0.05;
        // The point which you want to get the robot to.
        double ySetpoint = 0;
        double yILimit = 2;

        //calculations 

        // Figures out how long the function has been running.
        double timeInterval = Timer.getFPGATimestamp() - lastTimeStamp;
        
        //Finds the desired point by subtracting the current position from the set position
        double xError = xSetpoint - tx;
        
        //Finds the rate of change in error for use in the D portion of the PID loop.
        double xErrorRate = (xError - xLastError) / timeInterval;
        
        // The I constant increases over time to allow the robot to overcome friction and move the last little bit to get to the desired point.
        double xErrorSum = 0;
        if (Math.abs(xError) < xILimit) {
          xErrorSum = timeInterval * xError;
        }
        
        // P, I, and D
        double xP = xKP * xError;
        double xI = xKI * xErrorSum;
        double xD = xKD * xErrorRate;

        // The final output speed
        double xOutputSpeed = xP + xI + xD;
        
        // The error in x from the previous loop is used to calculate the rate of change in error.
        xLastError = xError;

        // The error form the desired point
        double yError = ySetpoint - ty;
        
        // Rate of change in error
        double yErrorRate = (yError - yLastError) / timeInterval;
        
        // The I constant increases over time to allow the robot to overcome friction and move the last little bit to get to the desired point.
        double yErrorSum = 0;
        if (Math.abs(yError) < yILimit) {
          yErrorSum = timeInterval * yError;
        } 
        
        double yP = yKP * yError;
        double yI = yKI * yErrorSum;
        double yD = yKD * yErrorRate;

        // The final output speed is the sum of P, I, and D
        double yOutputSpeed = yP + yI + yD;
        
        yLastError = yError;

        //combine x and y outputs
        double left = yOutputSpeed - xOutputSpeed;
        double right = yOutputSpeed + xOutputSpeed;

        //motor output
        drive.setPower(left, right);
      }
      else if (ballsHaveBeenShot == true) {

      }
    }

    double time = shootDelayTimer.get();

    if (-1 <= tx && tx <= 1 && -1 <=ty && ty <= 1) {
      shootDelayTimer.start();
      flyWheel.set(0.62);
      if (time >= 1 && time <=5) {
        drive.setPower(0,0);
        shooterIntakeActuator.set(kReverse); //extend actuator; down pos
        shooterIntake.set(-1);
        carousel.set(.32);
        ballsHaveBeenShot = true;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); //set limelight LED's to default pipeline setting (generally off)

      }
      else if (time >= 5 && time < 6) {
        
        shooterIntakeActuator.set(kForward);
        shooterIntake.set(0);
        carousel.set(0);
        flyWheel.set(0);
        drive.setPower(0, -0.2);
      }
      else if (time >=6 && time <8) {
        drive.setPower(0, 0);
      
        /*flDrive.set(ControlMode.PercentOutput, -.4);
        blDrive.set(ControlMode.PercentOutput, -.4);
        frDrive.set(ControlMode.PercentOutput, -.4);
        brDrive.set(ControlMode.PercentOutput, -.4);*/
      }
      /*else if (time >= 9) {
        flDrive.set(ControlMode.PercentOutput, 0);
        blDrive.set(ControlMode.PercentOutput, 0);
        frDrive.set(ControlMode.PercentOutput, 0);
        brDrive.set(ControlMode.PercentOutput, 0);
      }*/
      
    }
  }

  @Override
  public void teleopInit () {
    buttonIsHeld = 0;
  }

  @Override
  public void teleopPeriodic() {

// flywheel
    if (driveStick.getRawButton(1)) {flyWheel.set(0.62);} //shooting mode
    else if (driveStick.getRawButton(2)) {flyWheel.set(-.3);} //reversde mode: to clear jams
    else {flyWheel.set(0);}
// intake rollers
    if (buttonPad.getRawButton(12)) {intakeRollers.set(-1);}  //inverted, intaking
    else if (buttonPad.getRawButton(9)) {intakeRollers.set(1);} //"reversing", outputting
    else {intakeRollers.set(0);}
// intake arm
    if (buttonPad.getRawButton(10)) {intakeArm.set(-.5);} //inverted, deploying; doesnt need full power (especially since gravity helps)
    else if (buttonPad.getRawButton(7)) {intakeArm.set(.6);}  //retracting; little more power to get back up
    else {intakeArm.set(0);}
// carousel (indexer, wheel of fortune)
    if (buttonPad.getRawButton(11)) {carousel.set(.35);}  //CCW; slow down... will lose control, might break
    else if (buttonPad.getRawButton(8)) {carousel.set(-.35);} //CW
    else {carousel.set(0);}
// shooter intake, transitionary stage
    if (driveStick.getRawButton(3)) {
      shooterIntakeActuator.set(kReverse); //extend actuator; down pos
      shooterIntake.set(-1);}  //rveresed; intaking to shooter
    else if (driveStick.getRawButton(4))  {
      shooterIntakeActuator.set(kForward); //retract actuator; up pos
      shooterIntake.set(.7);}
    else {
      shooterIntakeActuator.set(kForward); //retract actuator; up pos
      shooterIntake.set(0);}
// climber winch
    if (buttonPad.getRawButton(2))  {winch.set(-1);}  //WINCH MUST ONLY SPIN BACKWARDS to save the motor and built in ratchet
    else {winch.set(0);}
//Control Panel (CP) Spinner
    if (povButtonUp.get()) {cpSpinnerActuator.set(kReverse);} //raises motor. Is held ther until the "down" button is pressed
    if (povButtonDown.get()) {cpSpinnerActuator.set(kForward);} //this is the down config and starting. only returns to this pos when button is pressed
    if (povButtonLeft.get()) {cpSpinner.set(-.2);}  //spin left
    else if (povButtonRight.get()) {cpSpinner.set(.2);} //spin right
    else {cpSpinner.set(0);}  //default state

    // Checks if the button is being held to prevent repeated overwriting of time.
    if (buttonIsHeld == 0 && buttonPad.getRawButton(1)) {
      lastTimeStamp = Timer.getFPGATimestamp();
      buttonIsHeld = 1;
    } else if (buttonIsHeld == 1 && buttonPad.getRawButton(1)) {

    } else {
      buttonIsHeld = 0;
    }

    //Drive control
    if (buttonPad.getRawButton(1)) {

      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //set LED's to forced on
      // The angle offset to the target on the horizontal axis.
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      // The angle offset to the target on the vertical axis.
      double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      
      //variable setup
      // Constant for P
      double xKP = -0.1;
      // Constant for I
      double xKI = 1;
      // Constant for D
      double xKD = 0.05;
      // The point you want to get the robot to.
      double xSetpoint = 0;
      double xILimit = 2;
      
      // Constant for P
      double yKP = -0.1;
      // Constant for I
      double yKI = 1;
      // Constant for D
      double yKD = 0.05;
      // The point which you want to get the robot to.
      double ySetpoint = 0;
      double yILimit = 2;

      //calculations 

      // Figures out how long the function has been running.
      double timeInterval = Timer.getFPGATimestamp() - lastTimeStamp;
      
      //Finds the desired point by subtracting the current position from the set position
      double xError = xSetpoint - tx;
      
      //Finds the rate of change in error for use in the D portion of the PID loop.
      double xErrorRate = (xError - xLastError) / timeInterval;
      
      // The I constant increases over time to allow the robot to overcome friction and move the last little bit to get to the desired point.
      double xErrorSum = 0;
      if (Math.abs(xError) < xILimit) {
        xErrorSum = timeInterval * xError;
      }
      
      // P, I, and D
      double xP = xKP * xError;
      double xI = xKI * xErrorSum;
      double xD = xKD * xErrorRate;

      // The final output speed
      double xOutputSpeed = xP + xI + xD;
      
      // The error in x from the previous loop is used to calculate the rate of change in error.
      xLastError = xError;

      // The error form the desired point
      double yError = ySetpoint - ty;
      
      // Rate of change in error
      double yErrorRate = (yError - yLastError) / timeInterval;
      
      // The I constant increases over time to allow the robot to overcome friction and move the last little bit to get to the desired point.
      double yErrorSum = 0;
      if (Math.abs(yError) < yILimit) {
        yErrorSum = timeInterval * yError;
      } 
      
      double yP = yKP * yError;
      double yI = yKI * yErrorSum;
      double yD = yKD * yErrorRate;

      // The final output speed is the sum of P, I, and D
      double yOutputSpeed = yP + yI + yD;
      
      yLastError = yError;

      //combine x and y outputs
      double left = yOutputSpeed - xOutputSpeed;
      double right = yOutputSpeed + xOutputSpeed;

      //motor output
      drive.setPower(left, right);
    }
    else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); //set limelight LED's to default pipeline setting (generally off)

      double speed = driveStick.getY();
      double turn = driveStick.getZ() * -.7;

      drive.arcadeDrive(speed, turn);
    }
  
  
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
