package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.Hardware.mmFTCFieldWidth;
import static org.firstinspires.ftc.teamcode.Hardware.mmTargetHeight;

/**
 * Created by User on 02/10/2018.
 */

public abstract class autoMain extends LinearOpMode {
    VuforiaLocalizer vuforia;
    VuforiaTrackable targetsRoverRuckus;
    VuforiaTrackables allTrackables;

    Hardware robot = new Hardware();   // Use a Pushbot's hardwareo

    public void Apolloinit(){
    robot.init(hardwareMap);
    initVuForia();
    }
    void ApolloRun (boolean isCreater){
        Apolloinit();
        readPhoto();
    //    moveCube(isCreater);
    //    enterCreater(isCreater);
    }
    void moveCube(boolean isCreater){

    }
    void enterCreater(boolean isCreater){

    }

    public void readPhoto() {

        while (opModeIsActive()) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                }
            }
        }
    }




public void initVuForia(){

    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    parameters.vuforiaLicenseKey = robot.VUFORIA_KEY ;
    parameters.cameraDirection   = robot.CAMERA_CHOICE;

    vuforia = ClassFactory.getInstance().createVuforia(parameters);
    // Load the data sets that for the trackable objects. These particular data
    // sets are stored in the 'assets' part of our application.
    VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
    VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
    blueRover.setName("Blue-Rover");
    VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
    redFootprint.setName("Red-Footprint");
    VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
    frontCraters.setName("Front-Craters");
    VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
    backSpace.setName("Back-Space");

    // For convenience, gather together all the trackable objects in one easily-iterable collection */

    //Target localization.
    OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
            .translation(0, -mmFTCFieldWidth, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
    redFootprint.setLocation(redFootprintLocationOnField);

    OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
            .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
    frontCraters.setLocation(frontCratersLocationOnField);

    OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
            .translation(mmFTCFieldWidth, 0, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
    backSpace.setLocation(backSpaceLocationOnField);

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

    }
}
