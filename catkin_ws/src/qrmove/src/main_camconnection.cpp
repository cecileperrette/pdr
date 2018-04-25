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

    // test OpenCV:
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }

    Mat image;
    image = imread( argv[1], 1 );

    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", image);

//    waitKey(0);

    // Start ROS loop:
    cout << "run CamConnection" << endl;
    ros::spin();
    // Properly stop the program:
    cout << "close CamConnection" << endl;

    return 0;
}
