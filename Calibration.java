package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import java.util.List;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
@Disabled
@Autonomous(name = "calibrate", group = "Concept")

public class Calibration extends LinearOpMode {
         BNO055IMU imu;
         BNO055IMU imu2;
          // NormalizedColorSensor colorSensor;
             double lastAngles;

    Orientation angles;
            double globalAngle;
           FairBotHardware robot           = new FairBotHardware();//links code to main hardware map
    private ElapsedTime runtime = new ElapsedTime();


    // values is a reference to the hsvValues array.
    //float[] hsvValues = new float[3];
    //final float values[] = hsvValues;

    // bPrevState and bCurrState keep track of the previous and current state of the button
    boolean bPrevState = false;
    boolean bCurrState = false;
double power=0;
double c_angle;
    // Get a reference to our sensor object.
   
           


@Override
    public void runOpMode() throws InterruptedException{
          robot.init(hardwareMap);
                       //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");




robot.frm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.flm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rrm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rlm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        int Distance;
         //Gyro
          BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
          
                 
        
        //GYRO
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(Gparameters);
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu2.initialize(Gparameters);       
        
        //GYRO
        while (!isStopRequested() && !imu.isGyroCalibrated() || !imu2.isGyroCalibrated())
        {

        telemetry.addData("Mode", "init");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("imu2 calib status", imu2.getCalibrationStatus().toString());
        telemetry.update();
        }
        Orientation angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation angles3 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
       

        telemetry.addData("Mode", "init");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("imu2 calib status", imu2.getCalibrationStatus().toString());
        telemetry.update();
        
        
        
        
        
        
        
        
        
        
        int Block = 1;
        int seen = 0;
        
                       
                       float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        
        // wait for the start button to be pressed.
        waitForStart();
                      
                     
                       
                  

      robot.claw.setPosition(.8);
    forward(22);
     sleep(500);
    turng(-90);
     sleep(500);
    forward(16);
    
    
double Block1=0;
double Block2=0;
      sleep(1000);
      if( Block1 + 2 > Block2 && Block2 + 2 > Block1){
          forward(11);
          turng(90);
          robot.claw.setPosition(.8);
          forward(6);
          sleep(100);
         // lift(2000);
          Block = 3;
      }
      else if(Block1 + 2 < Block2){
          forward(-8);
          turng(90);

          robot.claw.setPosition(.8);
          forward(6);
          sleep(100);
         // lift(2000);
          Block = 1;
      }
      else{//grab block 2
      forward(2);
          turng(90);    
      
          robot.claw.setPosition(.8);
          forward(6);
          sleep(100);
        //  lift(2000);
          Block = 2;
      }
      robot.claw.setPosition(.12);
      forward(-7);
      turng(90);
      forward((Block*8)+31);
      robot.claw.setPosition(.8);
      forward(-10);
      
      
      
      sleep(100000);
     }
      public void left(int Distance){
      
                   int new_d=Distance *53;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    //target position
    robot.frm.setTargetPosition(-new_d);
    robot.flm.setTargetPosition(new_d);
    robot.rrm.setTargetPosition(new_d);
    robot.rlm.setTargetPosition(-new_d);
    
    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rrm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rlm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // Set drive power
    robot.frm.setPower(-.25);
    robot.flm.setPower(-.25);
    robot.rrm.setPower(.25);
    robot.rlm.setPower(.25);
    
    while(opModeIsActive()&&robot.frm.isBusy() && robot.flm.isBusy() && robot.rrm.isBusy() && robot.rlm.isBusy()){

}
    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);


           
                      
    }
public void right(int Distance){
      
                   int new_d=Distance *53;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    //target position
    robot.frm.setTargetPosition(new_d);
    robot.flm.setTargetPosition(-new_d);
    robot.rrm.setTargetPosition(-new_d);
    robot.rlm.setTargetPosition(new_d);
    
    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rrm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rlm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // Set drive power
    robot.frm.setPower(-.25);
    robot.flm.setPower(-.25);
    robot.rrm.setPower(.25);
    robot.rlm.setPower(.25);
    
    while(opModeIsActive()&&robot.frm.isBusy() && robot.flm.isBusy() && robot.rrm.isBusy() && robot.rlm.isBusy()){
    
}
    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);


           
                      
    }
public void lift(int Distance){
    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
   // robot.lift.setTargetPosition(Distance);
    //robot.L2.setTargetPosition(Distance);
    
    robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.L2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
    
    
    if(Distance < 0){
    robot.lift.setPower(-1);
    robot.L2.setPower(-1);
    
    while(opModeIsActive()&&(robot.lift.getCurrentPosition()>Distance)&&(robot.L2.getCurrentPosition()>Distance)){}
    }
     else if(Distance > 0){
    robot.lift.setPower(1);
    robot.L2.setPower(1);
    
    while(opModeIsActive()&&(robot.lift.getCurrentPosition()<Distance)&&(robot.L2.getCurrentPosition()<Distance)){}
    }
    
    
   
    robot.lift.setPower(0);
    robot.L2.setPower(0);
}
public void extend(int Distance){
     robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    robot.winch.setTargetPosition(Distance);
    
    robot.winch.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
    robot.winch.setPower(-1);
    
    while(opModeIsActive()&&(robot.winch.getCurrentPosition()>Distance)){}
    robot.winch.setPower(0);
    
}
    public void turng(int Distance){

      double cDist = 0;
                   resetAngle();
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
   
    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
   while(opModeIsActive() && Math.abs(cDist) < Math.abs(Distance)){
       power = .6;
if( runtime.seconds() < 1){
    power*= (runtime.seconds()/1);
}
else{
    power=power;
}
    cDist = getAngle();

        telemetry.addData("Gyro", cDist);
    telemetry.update();
        double down = Math.abs(Distance)-Math.abs(cDist);
    if(down <60&&down>10&& power*((down-10)/40)>.2){
        power*=((down-10)/50);
    }
    else if(down<10 || power*((down-10)/40)<.2){
        power=.2;
        
    }
       
       
    if (Distance > 0){
    // Set drive power
    robot.frm.setPower(-power);
    robot.flm.setPower(power);
    robot.rrm.setPower(-power);
    robot.rlm.setPower(power);
    }
    else if (Distance < 0){
    // Set drive power
    robot.frm.setPower(power);
    robot.flm.setPower(-power);
    robot.rrm.setPower(power);
    robot.rlm.setPower(-power);
}
}

    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);


           
                      
    }
public void forward(int Distance){
      double rpower, lpower, ln, rn;
      resetAngle();
                   Distance =Distance *40;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    

    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
  runtime.reset();  
if(Distance >0){
 while(opModeIsActive()&&(robot.frm.getCurrentPosition()<Distance) && (robot.flm.getCurrentPosition()<Distance) && (robot.rrm.getCurrentPosition()<Distance) && (robot.rlm.getCurrentPosition()<Distance)){
if( runtime.seconds() < 1){
    rpower=.4 * (runtime.seconds());
    lpower=.4 * (runtime.seconds());
}
else{
    rpower=.4;
    lpower=.4;
}
    // Set drive power

    c_angle = getAngle();
       /* telemetry.addData("Right", robot.frm.getCurrentPosition());
        telemetry.addData("Left", robot.flm.getCurrentPosition());
        telemetry.addData("Rear Right", robot.rrm.getCurrentPosition());
        telemetry.addData("Real Left", robot.rlm.getCurrentPosition());
        telemetry.addData("Gyro", c_angle);*/
    telemetry.update();
    if(c_angle < -0.1){
        rn = rpower+(-c_angle/10);
        ln=lpower;
    }
    else if(c_angle > 0.1){
      ln = lpower+(c_angle/10);
      rn=rpower;
    }
    else{
        rn=rpower;
        ln=lpower;
    }
    
    robot.frm.setPower(rn);
    robot.flm.setPower(ln);
    robot.rrm.setPower(rn);
    robot.rlm.setPower(ln);
 }
}
else if(Distance<0){
   while(opModeIsActive()&&(robot.frm.getCurrentPosition()>Distance) && (robot.flm.getCurrentPosition()>Distance) && (robot.rrm.getCurrentPosition()>Distance) && (robot.rlm.getCurrentPosition()>Distance)){
if( runtime.seconds() < 1){
    rpower=-.6 * runtime.seconds();
    lpower=-.6 * runtime.seconds();
}
else{
    rpower=-.6;
    lpower=-.6;
}
    // Set drive power

    c_angle = getAngle();
     /*   telemetry.addData("Right", robot.frm.getCurrentPosition());
        telemetry.addData("Left", robot.flm.getCurrentPosition());
        telemetry.addData("Rear Right", robot.rrm.getCurrentPosition());
        telemetry.addData("Real Left", robot.rlm.getCurrentPosition());
        telemetry.addData("Gyro", c_angle);*/
    telemetry.update();
    if(c_angle < 0.1){
        rn = rpower+(c_angle/20);
        ln=lpower;
    }
    else if(c_angle > 0.1){
      ln = lpower+(c_angle/20);
      rn=rpower;
    }
    else{
        rn=rpower;
        ln=lpower;
    }
   robot.frm.setPower(rn);
    robot.flm.setPower(ln);
    robot.rrm.setPower(rn);
    robot.rlm.setPower(ln);
 }
   
}
    
    
    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);
   /* if((new_d-robot.rlm.getCurrentPosition())>10 || (new_d-robot.flm.getCurrentPosition())>10 ){
        robot.rlm.setPower(.5);
        robot.flm.setPower(.5);
    }
    else if((new_d-robot.rrm.getCurrentPosition())>10 || (new_d-robot.frm.getCurrentPosition())>10 ){
        robot.rrm.setPower(.5);
        robot.frm.setPower(.5);
    }

    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);*/
           
                      
    }
public void turn(int Distance){
      
                   Distance=Distance *10;
    robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
   
    //set to Run_TO_POSITION OpMode
    robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    if (Distance > 0){
        
    
    // Set drive power
    robot.frm.setPower(-.3);
    robot.flm.setPower(.3);
    robot.rrm.setPower(-.3);
    robot.rlm.setPower(.3);
    
    while(opModeIsActive()&&(-robot.frm.getCurrentPosition()<Distance) && (robot.flm.getCurrentPosition()<Distance) && (-robot.rrm.getCurrentPosition()<Distance) && (robot.rlm.getCurrentPosition()<Distance)){

}}

    else if (Distance < 0){
        
    
    // Set drive power
    robot.frm.setPower(.3);
    robot.flm.setPower(-.3);
    robot.rrm.setPower(.3);
    robot.rlm.setPower(-.3);
    
    while(opModeIsActive()&&(-robot.frm.getCurrentPosition()>Distance) && (robot.flm.getCurrentPosition()>Distance) && (robot.rrm.getCurrentPosition()>Distance) && (-robot.rlm.getCurrentPosition()>Distance)){

}}


    robot.frm.setPower(0);
    robot.flm.setPower(0);
    robot.rrm.setPower(0);
    robot.rlm.setPower(0);


           
                      
    }
public void resetAngle() {
        Orientation angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation angles3 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        lastAngles = (angles2.firstAngle+angles3.firstAngle)/2;
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()   {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation angles3 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        
        double deltaAngle = ((angles2.firstAngle+angles3.firstAngle)/2) - lastAngles;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = ((angles2.firstAngle+angles3.firstAngle)/2);

        return globalAngle;
    }

  /*protected void GetColor() throws InterruptedException {

    // If possible, turn the light on in the beginning (it might already be on anyway,
    // we just make sure it is if we can).
    if (colorSensor instanceof SwitchableLight) {
      ((SwitchableLight)colorSensor).enableLight(true);
    }


    // Loop until we are asked to stop
    while (opModeIsActive()) {
      // Check the status of the x button on the gamepad
      bCurrState = gamepad1.x;

      // If the button state is different than what it was, then act
      if (bCurrState != bPrevState) {
        // If the button is (now) down, then toggle the light
        if (bCurrState) {
          if (colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight)colorSensor;
            light.enableLight(!light.isLightOn());
          }
        }
      }
      bPrevState = bCurrState;

      // Read the sensor
      NormalizedRGBA colors = colorSensor.getNormalizedColors();

      /** Use telemetry to display feedback on the driver station. We show the conversion
       * of the colors to hue, saturation and value, and display the the normalized values
       * as returned from the sensor.
       * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/
/*
      Color.colorToHSV(colors.toColor(), hsvValues);
      telemetry.addLine()
              .addData("H", "%.3f", hsvValues[0])
              .addData("S", "%.3f", hsvValues[1])
              .addData("V", "%.3f", hsvValues[2]);
      telemetry.addLine()
              .addData("a", "%.3f", colors.alpha)
              .addData("r", "%.3f", colors.red)
              .addData("g", "%.3f", colors.green)
              .addData("b", "%.3f", colors.blue);




}
}*/
}
