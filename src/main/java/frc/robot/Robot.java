
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.DriverStation; 

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {
    private DifferentialDrive m_differentialDrive; //Talks to a right & left Talon which talks to motors (motor ctrl)
    private XboxController m_primaryController;
    private NetworkTableInstance ntInstance;
    NetworkTable motorTable; //Displays info on driver ctrl screen
    // private Talon m_intakeMotor;

    @Override
    public void robotInit() { // once the robot has turned on what will it do?
        // Setup XBox controller
        m_primaryController = new XboxController(0);

        // Setup drive motors
        m_differentialDrive = new DifferentialDrive(new PWMTalonSRX(8), new PWMTalonSRX(9));

        // Start camera server
        try{
            UsbCamera cameraFront = CameraServer.getInstance().startAutomaticCapture(0);
            UsbCamera cameraBack  = CameraServer.getInstance().startAutomaticCapture(1);
        // Set the camera video mode
            VideoMode[] modeFront = cameraFront.enumerateVideoModes();
            VideoMode[] modeBack = cameraFront.enumerateVideoModes();
  
            cameraFront.setVideoMode(modeFront[100]);
            cameraBack.setVideoMode(modeBack[100]);
      }
      catch (Exception e) 
      {
        System.err.println("No USB Camera Found");
      }
  
    }

    @Override
    public void teleopPeriodic() {
        arcadeDrive();

        // climbing
        // ball delivery
        // control panel
        // cameras
        // winch open
        // winch closed
        // driving
        // ball intke - motor
        // flap
        // control pannel motor
                //ask field management system what colour
    }

    public void arcadeDrive() {
        // Arcade drive motor control
        var forward = m_primaryController.getY(Hand.kLeft) * 0.8;
        var spin = m_primaryController.getX(Hand.kRight) * 0.45;
        m_differentialDrive.arcadeDrive(forward, spin, false);
    }
    public void climbingsubsystem() {
        // talk to two controllers (three motors) - 1st controller twinMotorController, 2nd motor singleMotorController
        // when twins go forward, single goes backward - controlled by 'up' on dPad
        // when twins go backward, single goes forward - controlled by 'down' on dPad
        var twinMotorController = new PWMTalonSRX(0);
        var singleMotorController = new PWMTalonSRX(1);
        var isPressed = m_primaryController.getPOV();
        switch (isPressed){
            case 0:
            // raise climbing mech
            twinMotorController.set(0.4);
            singleMotorController.set(-0.4);
            case 180:
            // lower climbing mech
            twinMotorController.set(-0.4);
            singleMotorController.set(0.4);
            default:
            twinMotorController.set(0);
            singleMotorController.set(0);
            return;
        }
    }
    public void ballmech(){
        //one motor to controll conveyor system?
        //flap control??
    }
    public void controlpanelcontrol(){
        //spinner on top controlled by motor 
        //colour sensor 
        //control panel number of rotations
        rotatepanel(3); // ?
        //control panel planned colour 
    }
    public void rotatepanel(int numberofrotations){
        //spin control panel 'numberofrotations' times
        
    }
    public void movetosetcolour(Colour colour){
        //detects current colour sensor reading 
        //compare it to what it should be 
        //spin wheel 
    }
    public Colour getcolour(){
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'B' :
                return Colour.Blue;
                case 'G' :
                return Colour.Green;
                case 'R' :
                return Colour.Red;
                case 'Y' :
                return Colour.Yellow;

            }
        }
        return Colour.NotSet;
    }

    enum Colour
    {
        NotSet,
        Blue,
        Red,
        Yellow,
        Green;
    }
}
