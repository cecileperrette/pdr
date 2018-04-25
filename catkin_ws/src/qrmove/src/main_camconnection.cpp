#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    // ROS:
    cout << "Initialize QRMove::CamConnection: " << endl;
    ros::init( argc, argv, "camconnection" );
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    // test OpenCV dans ROS:

     // Image
      IplImage *image;
     // Capture vidéo
     CvCapture *capture;

     // Ouvrir le flux vidéo
     capture = cvCreateCameraCapture(CV_CAP_ANY);
     // modifier la valeur CV_CAP_ANY pour atteindre une autre camera.

     // Vérifier si l'ouverture du flux est ok
     if (!capture) {
        printf("Ouverture du flux vidéo impossible !\n");
        return 1;
     }

     // Définition de la fenêtre
     cvNamedWindow("GeckoGeek Window", CV_WINDOW_AUTOSIZE);

     // Start ROS loop:
     cout << "run CamConnection" << endl;
     while( ros::ok() )
     {
         // On récupère une image
        image = cvQueryFrame(capture);

        // On affiche l'image dans une fenêtre
        cvShowImage( "GeckoGeek Window", image);
        cvWaitKey(5);

        ros::spinOnce();
     }

    // Properly stop the program:
    cvReleaseCapture(&capture);
    cvDestroyWindow("GeckoGeek Window");

    cout << "close CamConnection" << endl;

    return 0;
}
