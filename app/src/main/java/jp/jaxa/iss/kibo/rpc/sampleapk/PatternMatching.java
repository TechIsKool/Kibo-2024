package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

public class PatternMatching extends KiboRpcService {
    @Override
    protected void runPlan1(){
        // The mission starts.
        api.startMission();

        // Move to a point.
        Point point = new Point(10.9d, -9.92284d, 5.21d);
        Quaternion quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        api.moveTo(point, quaternion, true);

        // Get a camera image.
        Mat image = api.getMatNavCam();
        api.saveMatImage(image, "hi");

        org.opencv.core.Point matchLoc = new org.opencv.core.Point();
        Core.MinMaxLocResult mmlr = new Core.MinMaxLocResult();
        /* *********************************************************************** */
        /* Write your code to recognize type and number of items in the each area! */
        /* *********************************************************************** */

        // Detect AR
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Aruco.detectMarkers(image, dictionary, corners, markerIds);

        // Get camera matrix
        Mat  cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);

        // Get Lens distortion parameters
        Mat cameraCoefficients = new Mat(1, 5, CvType.CV_64F);
        cameraCoefficients.put(0, 0, api.getNavCamIntrinsics()[1]);
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);

        // Undistort image
        Mat undistortImg = new Mat();
        Calib3d.undistort(image, undistortImg, cameraMatrix, cameraCoefficients);

        String[] TEMPLATE_FILE_NAME = {"beaker.png", "goggle.png", "hammer.png", "kapton_tape.png", "pipette.png", "screwdriver.png", "thermometer.png", "top.png", "watch.png", "wrench.png"};
        String[] TEMPLATE_NAME = {"beaker", "goggle", "hammer", "kapton_tape", "pipette", "screwdriver", "thermometer", "top", "watch", "wrench"};
        // Pattern matching
        // Load template images
        Mat[] templates = new Mat[TEMPLATE_FILE_NAME.length];
        for (int i = 0; i < TEMPLATE_FILE_NAME.length; i++){
            try {
                // Open the template image file in Bitmap from the file name and convert to Mat
                InputStream inputStream = getAssets().open(TEMPLATE_FILE_NAME[i]);
                Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                Mat mat = new Mat();
                Utils.bitmapToMat(bitmap, mat);

                //Convert to grayscale
                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);

                // Assign to an  array  of templates
                templates[i] = mat;

                inputStream.close();

            }catch (IOException e){
                e.printStackTrace();
            }
        }

        // Number of  matches for each template
        int templateMatchCnt[] = new int[10];

        // Get the number of template matches
        for (int tempNum = 0; tempNum < templates.length; tempNum++){
            // Number of matches
            int matchCnt  = 0;

            // Coordinates of the matched locatioin
            List<org.opencv.core.Point> matches = new ArrayList<>();

            // Loading template image and target image
            Mat template = templates[tempNum].clone();
            Mat targetImg = undistortImg.clone();

            // Pattern matching
            int widthMin  = 20; //[px]
            int widthMax = 100; //[px]
            int changeWidth = 5; //[px]
            int changeAngle = 45; //[degree]

            for (int i = widthMin; i <= widthMax; i += changeWidth){
                for (int j = 0; j <= 360; j += changeAngle){
                    Mat resizedTemp = resizeImg(template, i);
                    Mat rotResizedTemp = rotImg(resizedTemp, j);
                    api.saveMatImage(rotResizedTemp, "no");
                    Mat result = new Mat();
                    Imgproc.matchTemplate(targetImg, rotResizedTemp, result, Imgproc.TM_CCOEFF_NORMED);
                    // Get  coordinates with similarity grater than or equal to the threshold
                    double threshold = 0.8;
                    mmlr = Core.minMaxLoc(result);

                    double maxVal = mmlr.maxVal;
                    if (maxVal >= threshold){
                        // Extract only results grater than or equal to the threshold
                        Mat thresholdedResult = new Mat();
                        Imgproc.threshold(result, thresholdedResult, threshold, 1.0, Imgproc.THRESH_TOZERO);

                        // Get match counts
                        for (int y = 0; y < thresholdedResult.rows(); y++){
                            for (int x = 0; x < thresholdedResult.cols(); x++){
                                if (thresholdedResult.get(y, x)[0] > 0){
                                    matches.add(new org.opencv.core.Point(x, y));
                                }
                            }
                        }
                    }
//                    else {
//                        System.out.println("Template/Image not detected");
//                    }
                }
            }
            // Avoid detecting the same location multiple times
            List<org.opencv.core.Point> filteredMatches = removeDuplicates(matches);
            matchCnt = filteredMatches.size();

            // Number of matches for each template
            templateMatchCnt[tempNum] = matchCnt;
        }

        // turn on the front flash light
//        api.flashlightControlFront(0.05f);

        // get QR code content
        String mQrContent = yourMethod();

        // turn off the front flash light
//        api.flashlightControlFront(0.00f);

        // When you recognize items, let’s set the type and number.
        int mostMatchTemlatenNum = getMaxIndex(templateMatchCnt);
        Mat img = templates[mostMatchTemlatenNum].clone();
        // Locate the match location
        matchLoc = mmlr.maxLoc;

        // Draw rectangle around the detected object
        Imgproc.rectangle(
                undistortImg,
                matchLoc,
                new org.opencv.core.Point(matchLoc.x + img.cols(), matchLoc.y + img.rows()),
                new Scalar(0, 255, 0),
                2
        );

        api.saveMatImage(undistortImg, "Final");

        System.out.println("name : "+ TEMPLATE_NAME[mostMatchTemlatenNum] + " number of images : " + templateMatchCnt[mostMatchTemlatenNum]);
        api.setAreaInfo(1, TEMPLATE_NAME[mostMatchTemlatenNum], templateMatchCnt[mostMatchTemlatenNum]);

        /* **************************************************** */
        /* Let's move to the each area and recognize the items. */
        /* **************************************************** */

//        Point point2 = new Point(11.05d, -9.00d, 5.13d);
//        Quaternion quaternion2 = new Quaternion(0f, 0.707f, 0f, 0.707f);
//        api.moveTo(point2, quaternion2, true);

        // When you move to the front of the astronaut, report the rounding completion.
        api.reportRoundingCompletion();

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
    public String yourMethod(){
        return "your method";
    }

    // Resize image
    private Mat resizeImg (Mat img, int width){
        int heigth = (int) (img.rows() * (double) width / img.cols());
        Mat resizedImg = new Mat();
        Imgproc.resize(img, resizedImg, new Size(width, heigth));

        return resizedImg;
    }

    // Rotate image
    private Mat rotImg (Mat img, int angle){
        org.opencv.core.Point center = new org.opencv.core.Point(img.cols() / 2.0, img.rows() / 2.0);
        Mat rotatedMat = Imgproc.getRotationMatrix2D(center, angle, 1.0);
        Mat rotatedImg = new Mat();
        Imgproc.warpAffine(img, rotatedImg, rotatedMat, img.size());

        return rotatedImg;
    }

    // Remove multiple detections
    private static List<org.opencv.core.Point> removeDuplicates (List<org.opencv.core.Point> points){
        double length = 10; // Width 10 px
        List<org.opencv.core.Point> filteredList = new ArrayList<>();

        for (org.opencv.core.Point point : points){
            boolean isInclude = false;
            for (org.opencv.core.Point checkPoint : filteredList){
                double distance = calculateDistance(point, checkPoint);
                if (distance <= length){
                    isInclude = true;
                    break;
                }
            }

            if (!isInclude){
                filteredList.add(point);
            }
        }
        return filteredList;
    }

    // Find the distance between two points
    private static double calculateDistance(org.opencv.core.Point p1, org.opencv.core.Point p2){
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

    }

    // Get the maximum value of an array
    private int getMaxIndex (int[] array){
        int max = 0;
        int maxIndex = 0;

        // Find the index of the element with the largest value
        for (int i = 0; i < array.length; i++){
            if (array[i] > max) {
                max = array[i];
                maxIndex = i;
            }
        }
        return maxIndex;
    }
}
