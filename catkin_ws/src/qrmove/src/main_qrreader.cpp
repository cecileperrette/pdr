#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <zbar.h>
#include "opencv2/videoio.hpp"

#include <iostream>

using namespace cv;
using namespace std;
using namespace zbar;

int main(int argc, char* argv[])
{

	// ROS:
    cout << "Initialize QRMove::CamConnection: " << endl;
    ros::init( argc, argv, "camconnection" );
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    // test OpenCV dans ROS:
	cv_bridge::CvImagePtr cv_ptr;

   // Capture vidéo
     CvCapture *capture;

	// Image
      IplImage *image;

     // Ouvrir le flux vidéo
     capture = cvCreateCameraCapture(CV_CAP_ANY);
     // modifier la valeur CV_CAP_ANY pour atteindre une autre camera.

     // Vérifier si l'ouverture du flux est ok
     if (!capture) {
        printf("Ouverture du flux vidéo impossible !\n");
        return 1;
     }

   ImageScanner scanner;
   scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

   double dWidth = cv_ptr(CV_CAP_PROP_FRAME_WIDTH); //Pour avoir la largeur de l'écran
   double dHeight = cv_ptr(CV_CAP_PROP_FRAME_HEIGHT); //Pour avoir la longeur de l'écran

   cout << "Frame size : " << dWidth << " x " << dHeight << endl;

   namedWindow("QRSearch", CV_WINDOW_AUTOSIZE); //Création d'une fenêtre

   while (ros::ok())
   {
      Mat frame=cv_ptr->image;
      }
      Mat grey;
      cvtColor(frame, grey, CV_BGR2GRAY);

      int width = frame.cols;
      int height = frame.rows;
      uchar *raw = (uchar *)grey.data;

	Image image(width,height,"Y800",raw,width*height);	

      // On la scan à la recherche de qr code
      int n = scanner.scan(image);

      // On extrait les résultats
      for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
      {
         vector<Point> vp;

         cout << "decoded " << symbol->get_type_name() << " symbol \"" << symbol->get_data() << "\"" << endl;

	//On essaye de situer le QRCode
        int n = symbol->get_location_size();
        for(int i=0;i<n;i++)
        {
           vp.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
        }

	//On l'entoure
        RotatedRect r = minAreaRect(vp);
        Point2f pts[4];
        r.points(pts);

        for(int i=0;i<4;i++){
          line(frame, pts[i], pts[(i+1)%4], Scalar(255,0,0), 3);
        }

        cout<<"Angle: "<<r.angle<<endl;
     }

     imshow("QRSearch", frame); //Faire apparaitre le contour


     if (waitKey(30) == 27) //Condition d'arrêt
     {
         cout << "esc key is pressed by user" << endl;
         break;
     }
   }
   return 0;
}
