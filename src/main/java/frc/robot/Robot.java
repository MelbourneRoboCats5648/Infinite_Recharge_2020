
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.first.wpilibj.DriverStation;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * 
 * it contains the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {
    private DifferentialDrive m_differentialDrive; // Talks to a right & left Talon which talks to motors (motor ctrl)
    private XboxController m_primaryController;
    private NetworkTableInstance ntInstance;
    // motor control
    private VictorSP ball_mech_intake; //ball mech Intake - Victor SP port 4
    private VictorSP ball_mech_flap;  // Ball mech Flap - Victor SP port 7
    private PWMTalonSRX twinMotorController; // Climb Lift - Talon SRX port 8
    private VictorSP singleMotorController; // Climb extend - Victor SP port 3
    private VictorSP ctrlpanelmotor;// Control pannel - Victor SP port 6
    private PWMTalonSRX ball_mech_conveyer; // Ball mech conveyer - talon srx port 5
    private boolean startSpin = false;
    private boolean Start_search = false;
    private Colour startingcolour = Colour.NotSet;
    private Colour prevColour = Colour.NotSet;
    private Colour robot_sensor;
    private Colour converted_colour;
    private Colour gameSensor_target;
    private UsbCamera cameraFront;
    private UsbCamera cameraBack;
    private MjpegServer cameraServer;
    private Camera currentCamera;
    private double flapSpeed = 0.2;
    private double controlPanelRotationSpeed = 0.75;
    private double controlPanelSearchSpeed = 0.25;
    private double ballIntakeSpeed = 0.4;
    private double conveyorSpeed = 0.5;
    private double climbSpeed =  0.8;
    private Timer timer;


    private boolean cancel = false;
    int n = 0;

    NetworkTable motorTable; // Displays info on driver ctrl screen
    // private Talon m_intakeMotor;

    /**
     * Change the I2C port below to match the connection of your color sensor
     */
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
     * The device will be automatically initialized with default parameters.
     */
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void disabledInit() {
        super.disabledInit();
    }

    @Override
    public void robotInit() { // once the robot has turned on what will it do?
        // Setup XBox controller
        m_primaryController = new XboxController(0);

        // Setup drive motors
        m_differentialDrive = new DifferentialDrive(new PWMVictorSPX(2), new PWMVictorSPX(1)); // Victor SPS on ports 1&2

        // Setup network tables
        ntInstance = NetworkTableInstance.getDefault();

        SetupCameras();

        ctrlpanelmotor = new VictorSP(6);
        twinMotorController = new PWMTalonSRX(8);
        singleMotorController = new VictorSP(3);
        ball_mech_intake = new VictorSP(7);
        ball_mech_flap = new VictorSP(4);
        ball_mech_conveyer = new PWMTalonSRX(5);
    }

    public void SetupCameras()
    {        
        // Start camera server
        try {  
            cameraFront = CameraServer.getInstance().startAutomaticCapture(0);
            cameraBack = CameraServer.getInstance().startAutomaticCapture(1);
            cameraFront.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            cameraBack.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            
            // Set the camera video mode
            VideoMode[] modeFront = cameraFront.enumerateVideoModes();
            VideoMode[] modeBack = cameraFront.enumerateVideoModes();
            cameraFront.setVideoMode(modeFront[100]);
            cameraBack.setVideoMode(modeBack[100]);

            cameraServer = CameraServer.getInstance().addSwitchedCamera("switched camera");
        } catch (Exception e) {
            System.err.println("No USB Camera Found");
        }

        currentCamera = Camera.Front;
        cameraServer.setSource(cameraFront);
    }

    @Override
    public void autonomousInit(){
        timer = new Timer();
        timer.start();

        // BLEH
        // Drive forward - cross initiation line
        // Maybe shoot??

    }
    @Override
    public void autonomousPeriodic() {
        if (timer.get() < 5.0){
            m_differentialDrive.arcadeDrive(0.4, 0, false);
        }
        else{
            m_differentialDrive.arcadeDrive(0, 0, false);
        }
    }

    @Override
    public void teleopPeriodic() {
        switchCameras();
        arcadeDrive();

        climbingsubsystem();
        ballmech();
        controlpanelcontrol();        
    }

    public void switchCameras()
    {
        if(m_primaryController.getBumperPressed(Hand.kLeft))
        {
            if(currentCamera == Camera.Front)
            {
                cameraServer.setSource(cameraBack);
                currentCamera = Camera.Back;
            } else {
                cameraServer.setSource(cameraFront);
                currentCamera = Camera.Front;
            }
        }
    }

    public Colour DetectingRGBYfrmsensor() {
        // Colour sensor code
        /**
         * The method GetColor() returns a normalized color value from the sensor and
         * can be useful if outputting the color to an RGB LED or similar. To read the
         * raw color, use GetRawColor().
         * 
         * The color sensor works best when within a few inches from an object in well
         * lit conditions (the built in LED is a big help here!). The farther an object
         * is the more light from the surroundings will bleed into the measurements and
         * make it difficult to accurately determine its color.
         */
        Color detectedColor = m_colorSensor.getColor();

        /**
         * The sensor returns a raw IR value of the infrared light detected.
         */
        double IR = m_colorSensor.getIR();

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);

        // RLI = Reflected Light Intensity
        double red_RLI = detectedColor.red;
        double green_RLI = detectedColor.green;
        double blue_RLI = detectedColor.blue;

        // System.out.println(red_RLI);

        if (red_RLI > 0.6) {
            // Detected Red
            System.out.println("Red :)");
            return Colour.Red;
        }

        if (red_RLI > 0.4 && green_RLI > 0.485) {
            // Detected yellow
            System.out.println("Yellow :/");
            return Colour.Yellow;
        }

        if (green_RLI > 0.5 && blue_RLI > 0.19) {
            // Detected green
            System.out.println("Green :(");
            return Colour.Green;
        }

        if (blue_RLI > 0.3) {
            // Detected blue
            System.out.println("blue >_<");
            return Colour.Blue;
        }

        /**
         * In addition to RGB IR values, the color sensor can also return an infrared
         * proximity value. The chip contains an IR led which will emit IR pulses and
         * measure the intensity of the return. When an object is close the value of the
         * proximity will be large (max 2047 with default settings) and will approach
         * zero when the object is far away.
         * 
         * Proximity can be used to roughly approximate the distance of an object or
         * provide a threshold for when an object is close enough to provide accurate
         * color values.
         */
        int proximity = m_colorSensor.getProximity();
        return Colour.NotSet;
    }

    public void arcadeDrive() {
        // Arcade drive motor control
        var forward = m_primaryController.getY(Hand.kLeft) * 0.8;
        var spin = m_primaryController.getX(Hand.kRight) * 0.45;
        m_differentialDrive.arcadeDrive(forward, spin, false);
    }

    public void climbingsubsystem() {
        // talk to two controllers (three motors) - 1st controller twinMotorController,
        // 2nd motor singleMotorController
        // when twins go forward, single goes backward - controlled by 'up' on dPad
        // when twins go backward, single goes forward - controlled by 'down' on dPad
        if (Cancel()) {
            return;
        }
        var isPressed = m_primaryController.getPOV();
        switch (isPressed) {
        case 0:
            // raise climbing mech
            twinMotorController.set(-climbSpeed*1.02);
            singleMotorController.set(-climbSpeed);
            return;
        case 180:
            // lower climbing mech
            twinMotorController.set(climbSpeed*1.03); // multiplying to account for different motor speed
            singleMotorController.set(climbSpeed);
            return;
        default:
            twinMotorController.set(0);
            singleMotorController.set(0);
            return;
        }
    }

    public void ballmech() {
        // one motor to controll conveyor system?
        // flap control??
        // ball mech intake controlls two motors one is wired back to front which goes
        // backward wen the other goes forward
        if (Cancel()) {
            return;
        }
        //intake code: 
        if (m_primaryController.getBButtonPressed()){
            if (ball_mech_intake.getSpeed() == 0) {
                ball_mech_intake.set(-ballIntakeSpeed);
                ball_mech_conveyer.set(conveyorSpeed);
            }
            else {
                ball_mech_intake.set(0);
                ball_mech_conveyer.set(0);
            }
        }
        if (m_primaryController.getAButtonPressed()){
            if (ball_mech_intake.getSpeed() == 0) {
                ball_mech_intake.set(ballIntakeSpeed);
                ball_mech_conveyer.set(-conveyorSpeed);
            }
            else {
                ball_mech_intake.set(0);
                ball_mech_conveyer.set(0);
            }
        }
        // flap control code:
        if (m_primaryController.getTriggerAxis(Hand.kLeft) < 0.1 && m_primaryController.getTriggerAxis(Hand.kRight) < 0.1)
        {
            ball_mech_flap.set(0);
            return;
        }
        // For lowering think
        ball_mech_flap.set(flapSpeed*m_primaryController.getTriggerAxis(Hand.kLeft));
        //For raising think
        ball_mech_flap.set(-flapSpeed*m_primaryController.getTriggerAxis(Hand.kRight));

    }

    public void controlpanelcontrol() {
        // spinner on top controlled by motor
        // colour sensor
        // control panel number of rotations
        rotatepanel(3); // ?
        // control panel planned colour
        movetosetcolour();
    }

    public void rotatepanel(int numberofrotations) {
        // spin control panel 'numberofrotations' times
        var detectColourNum = numberofrotations * 2 + 1;
        if (Cancel()) {
            startSpin = false;
            return;
        }
        if (startSpin == false) {
            if (m_primaryController.getXButton()) {
                startingcolour = DetectingRGBYfrmsensor(); // reading the starting colour into a variable
                ctrlpanelmotor.set(controlPanelRotationSpeed);
                startSpin = true;
                prevColour = startingcolour;
                n = 0;
            }
        }

        if (startSpin == true) {
            var currentColour = DetectingRGBYfrmsensor();
            if (currentColour != prevColour) {
                if (currentColour == startingcolour) {
                    n = n+1;
                }
            }
            prevColour = currentColour;
            if (n == detectColourNum) {
                startSpin = false;
                ctrlpanelmotor.set(0);
            }
        }        
    }

    /**
     * detects current colour sensor reading
     * compare it to what it should be
     * spin wheel
     */
    public void movetosetcolour() {
        if (Cancel()) {
            Start_search = false;
            return;
        }

        if (m_primaryController.getYButtonPressed()) {
            Start_search = true;
            gameSensor_target = getcolour(); // defining what colour we are looking for under game sensor
        }
        if (Start_search == true) {
            robot_sensor = DetectingRGBYfrmsensor();
            converted_colour = robotToGameColour(robot_sensor);
            if (converted_colour != gameSensor_target) {
                ctrlpanelmotor.set(controlPanelSearchSpeed);
            } else {
                ctrlpanelmotor.set(0);
                Start_search = false;
            }
        }
    }         
    
    public boolean Cancel(){
        boolean is_cancel_pressed = m_primaryController.getStartButtonPressed();
        if (is_cancel_pressed) {
            ball_mech_intake.set(0);
            ball_mech_flap.set(0);
            twinMotorController.set(0);
            singleMotorController.set(0);
            ctrlpanelmotor.set(0);
            ball_mech_conveyer.set(0);
            return true;
        }
        else {
            return false;
        }
    }
    /**
     * Getting the game specific message and finding out what colour we are targeting
     * @return Colour from game specific msg
     */
    public Colour getcolour() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        // informing the driver station the colour you are looking for
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
            case 'B':
                return Colour.Blue;
            case 'G':
                return Colour.Green;
            case 'R':
                return Colour.Red;
            case 'Y':
                return Colour.Yellow;

            }
        }
        return Colour.NotSet;

    }

    public Colour robotToGameColour(Colour robotcolour) {
        if (robotcolour == Colour.Red){
            return Colour.Blue;
        }
        if (robotcolour == Colour.Blue){
            return Colour.Red;
        }
        if (robotcolour == Colour.Green){
            return Colour.Yellow;
        }
        if (robotcolour == Colour.Yellow){
            return Colour.Green;
        }
        return Colour.NotSet;
    }


    enum Colour {
        NotSet, Blue, Red, Yellow, Green;
    }

    enum Camera { Front, Back; }

    //@Override
    public void calibration() { //testPeriodic
        if (m_primaryController.getXButtonPressed()) {
            ball_mech_conveyer.set(1);
            ball_mech_flap.set(1);
            ball_mech_intake.set(1);
            twinMotorController.set(1);
            singleMotorController.set(1);
            ctrlpanelmotor.set(1);
            m_differentialDrive.arcadeDrive(1, 0, false);
        }
        if (m_primaryController.getYButtonPressed()) {
            ball_mech_conveyer.set(-1);
            ball_mech_flap.set(-1);
            ball_mech_intake.set(-1);
            twinMotorController.set(-1);
            singleMotorController.set(-1);
            ctrlpanelmotor.set(-1);
            m_differentialDrive.arcadeDrive(-1, 0, false);
        }
        if (m_primaryController.getBButtonPressed()) {
            ball_mech_conveyer.set(0);
            ball_mech_flap.set(0);
            ball_mech_intake.set(0);
            twinMotorController.set(0);
            singleMotorController.set(0);
            ctrlpanelmotor.set(0);
            m_differentialDrive.arcadeDrive(0, 0, false);
        }
    }

}
