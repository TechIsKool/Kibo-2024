package jp.jaxa.iss.kibo.rpc.sampleapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class Coordinates extends KiboRpcService {
    @Override
    protected void runPlan1(){
        // The mission starts.
        api.startMission();

        // Area 1 coords
        Point area1 = new Point(10.9d, -9.92284d, 5.195d);
        Quaternion quaternion1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        api.moveTo(area1, quaternion1, false);

        // Area 2 coords
        Point area2p1 = new Point(11.195d, -8.875d, 5.295d);
        Quaternion quaternion2 = new Quaternion(0, 0.707f, 0, 0.707f);
        api.moveTo(area2p1, quaternion2, true);

        Point area2p2 = new Point(10.925d, -8.875d, 4.55d);
        api.moveTo(area2p2, quaternion2, true);

        // Area 3 coords
        Point area3 = new Point(10.925d, -7.925d, 4.55d);
        api.moveTo(area3, quaternion2, true);

        // Area 4 coords
        Point area4p1 = new Point(10.56d, -7.35d, 4.62d);
        Quaternion quaternion3 = new Quaternion(0f, 0f, 1.0f,0);
        Quaternion quaternion5 = new Quaternion(0, 0, 0f, 0f);
        api.moveTo(area4p1, quaternion3, true);

        Point area4p2 = new Point(11.235d, -6.8525d, 4.945d);
        api.moveTo(area4p2, quaternion3, true);

        // Astrounut Cords
        Point astrounut = new Point(11.143d, -6.7d, 4.945d);
        Quaternion quaternion4 = new Quaternion(0, 0, 0.707f, 0.707f);
        api.moveTo(astrounut, quaternion4, true);

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
        Mat image = api.getMatNavCam();

        /* *********************************************************************** */
        /* Write your code to recognize type and number of items in the each area! */
        /* *********************************************************************** */

        // When you recognize items, let’s set the type and number.
        api.setAreaInfo(1, "item_name", 1);

        /* **************************************************** */
        /* Let's move to the each area and recognize the items. */
        /* **************************************************** */

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
    private String yourMethod(){
        return "your method";
    }
}