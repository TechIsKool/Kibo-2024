package jp.jaxa.iss.kibo.rpc.sampleapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        // The mission starts.
        api.startMission();

        // Area 1 coords
         Mat area_1 = moveToObject(10.9d, -9.92284d, 5.195d, 0f, 0f, -0.707f, 0.707f);
         api.saveMatImage(area_1, "Area 1");

        // Area 2 coords
        Point area2p1 = new Point(11.195d, -8.875d, 5.295d);
        Quaternion quaternion2 = new Quaternion(0, 0.707f, 0, 0.707f);
        api.moveTo(area2p1, quaternion2, true);

        Mat area_2 = moveToObject(10.925d, -8.875d, 4.55d, 0, 0.707f, 0, 0.707f);
        api.saveMatImage(area_2, "Area 2");

        // Area 3 coords
        Mat area_3 = moveToObject(10.925d, -7.925d, 4.55d, 0, 0.707f, 0, 0.707f);
        api.saveMatImage(area_3, "Area 3");

        // Area 4 coords
        Point area4p1 = new Point(10.56d, -7.35d, 4.62d);
        Quaternion quaternion3 = new Quaternion(0f, 0f, 1.0f,0);
        api.moveTo(area4p1, quaternion3, true);

        Mat area_4 = moveToObject(10.6d, -6.8525d, 4.945d, 0f, 0f, 1.0f,0);
        api.saveMatImage(area_4, "Area 4");

        api.setAreaInfo(1, "item_name", 1);

        // Astrounut Cords
        api.reportRoundingCompletion();
        Mat astrounut_cords = moveToObject(11.143d, -6.8d, 4.945d, 0, 0, 0.707f, 0.707f);
        api.saveMatImage(astrounut_cords, "Astrounut");

//        // Astrounut to Area 4
//        api.moveTo(area4p2, quaternion3, true);
//
//        // Astrounut to Area 3
//        api.moveTo(area4p2, quaternion3, true);
//        api.moveTo(area4p1, quaternion3, true);
//        api.moveTo(area3, quaternion2, true);
//
//        // Astrounut to Area 2
//        Point astro_area2p1 = new Point(11.235d, -7.4d, 5.295d);
//        api.moveTo(astro_area2p1, quaternion2, true);
//        Point astro_area2p2 = new Point(11.235d, -7.4d, 4.55d);
//        api.moveTo(astro_area2p2, quaternion2, true);
//        Point astro_area2p3 = new Point(10.925d, -8.875d, 4.55d);
//        api.moveTo(astro_area2p3, quaternion2, true);
//
//        // Astrounut to Area 1
//        Point astro_area1p1 = new Point(11.235d, -7.4d, 5.295d);
//        api.moveTo(astro_area1p1, quaternion1, true);
//        Point astro_area1p2 = new Point(11.235d, -8.5d, 4.62d);
//        api.moveTo(astro_area1p2, quaternion1, true);
//        api.moveTo(area2p1, quaternion2, true);
//        api.moveTo(area1, quaternion1, false);

        // Get a camera image.

        /* *********************************************************************** */
        /* Write your code to recognize type and number of items in the each area! */
        /* *********************************************************************** */

        // When you recognize items, let’s set the type and number.


        /* **************************************************** */
        /* Let's move to the each area and recognize the items. */
        /* **************************************************** */

        // When you move to the front of the astronaut, report the rounding completion.


        /* ********************************************************** */
        /* Write your code to recognize which item the astronaut has. */
        /* ********************************************************** */

        // Let's notify the astronaut when you recognize it.
        api.notifyRecognitionItem();

        /* ******************************************************************************************************* */
        /* Write your code to move Astrobee to the location of the target item (what the astronaut is looking for) */
        /* ******************************************************************************************************* */

        // Take a snapshot of the target item.
        api.takeTargetItemSnapshot();
    }

    @Override
    protected void runPlan2(){
        // write your plan 2 here.
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here.
    }

    // You can add your method.
    private Mat moveToObject(double xCord, double yCord, double zCord, float qx, float qy, float qz, float qw)
    {
        Mat cropped = new Mat();
        Point point = new Point(xCord, yCord, zCord);
        Quaternion quaternion = new Quaternion(qx, qy, qz, qw);
        api.moveTo(point, quaternion, false);
        try {
            Thread.sleep(3000);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
        Mat init_location = api.getMatNavCam();
        api.saveMatImage(init_location, "Initial Location");


        float markerLength = 0.05f;

        // Check if any markers are detected
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);
        Mat distCoeffs = new MatOfDouble(api.getNavCamIntrinsics()[1]);

        // Undistort Image
        Mat cameraCoefficients = new Mat( 1,  5, CvType.CV_64F);
        cameraCoefficients.put(  0,  0, api.getNavCamIntrinsics()[1]);
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);

        Mat undistortImg = new Mat();
        Calib3d.undistort(init_location, undistortImg, cameraMatrix, cameraCoefficients);
        api.saveMatImage(undistortImg, "Undistorted Image");

        Dictionary dictionary_2 = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners_2 = new ArrayList<>();
        Mat markerIds_2 = new Mat();
        Aruco.detectMarkers(undistortImg, dictionary_2, corners_2, markerIds_2);

        Mat rvecs = new Mat();
        Mat tvecs = new Mat();

        Aruco.estimatePoseSingleMarkers(corners_2, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

        Mat cornersMat = corners_2.get(0);

        System.out.println("Marker ID: " + markerIds_2.get(0, 0)[0]);
        double[] corner = cornersMat.get(0, 3);
        int corner_x = (int) corner[0];
        int corner_y = (int) corner[1];
        System.out.println("Corner of Undistorted Image: (" + corner_x + ", " + corner_y + ")");

        // Code for Rotating Undistorted Image
        double rotationAngle = calculateRotationAngle(rvecs);
        Mat rotatedImage = rotateImage(undistortImg, corner_x, corner_y, rotationAngle);
        api.saveMatImage(rotatedImage, "rotated image");


        // Code for cropping the rotated image
        Dictionary dictionary_3 = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners_3 = new ArrayList<>();
        Mat markerIds_3 = new Mat();
        Aruco.detectMarkers(rotatedImage, dictionary_3, corners_3, markerIds_3);

        Mat cornersMat_2 = corners_3.get(0);

        System.out.println("Marker ID: " + markerIds_3.get(0, 0)[0]);
        double[] corner_2 = cornersMat_2.get(0, 3);
        int corner_2_x = (int) corner_2[0];
        int corner_2_y = (int) corner_2[1];
        System.out.println("Corner: (" + corner_2_x + ", " + corner_2_y + ")");

        if (init_location.empty()) {
            System.out.println("Could not open or find the image");
        }
        Rect roi = new Rect(corner_2_x-195, corner_2_y-110, 280, 200);

        // Crop the image using the defined ROI
        cropped = new Mat(rotatedImage, roi);
        System.out.println("Image cropped and saved successfully");

        return cropped;
    }

    private static double calculateRotationAngle(Mat rvecs) {
        // Extract the rotation vector for the first detected marker
        Mat rvec = rvecs.row(0);

        // Convert the rotation vector to a rotation matrix
        Mat rotationMatrix = new Mat();
        Calib3d.Rodrigues(rvec, rotationMatrix);

        // Extract the angle of rotation around the Z-axis from the rotation matrix
        // The rotation matrix is a 3x3 matrix, we need to get the angle from it
        double angle = Math.atan2(rotationMatrix.get(1, 0)[0], rotationMatrix.get(0, 0)[0]);

        // Convert the angle to degrees
        return Math.toDegrees(angle);
    }

    private static Mat rotateImage(Mat image, int aruco_x, int aruco_y, double angle) {
        // Rotate the image by specified angle
        Mat rotatedImage = new Mat();
        org.opencv.core.Point center = new org.opencv.core.Point(aruco_x, aruco_y);
        Mat rotationMatrix = Imgproc.getRotationMatrix2D(center, angle, 1.0);
        Imgproc.warpAffine(image, rotatedImage, rotationMatrix, image.size());
        return rotatedImage;
    }
}