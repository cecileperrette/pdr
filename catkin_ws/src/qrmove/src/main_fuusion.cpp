#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <stdio.h>
#include <iostream>     // std::cout
#include <cmath>        // std::abs
#include <stdlib.h>

using namespace std;
using namespace cv;
using namespace zbar;

// On commence par définir la structure qui renfermera l'information concernant un QRCode détécté dans l'image donnée

typedef struct{
	string type;
	string data;
	vector <Point> location;
} decodedObject;

// on écrit une fonction qui va repérer des QRCodes

void decode(Mat &im, vector<decodedObject> &decodedObjects){
	
	// création d'un scanner à QRCode
	ImageScanner scanner;

	// Configuration du scanner
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE,1);

	// On converti l'image en nuances de gris
	Mat imGray;
	cvtColor(im,imGray,CV_BGR2GRAY);

	//on met l'image au format d'une image zbar pour une meilleure compatibilité
	Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data,im.cols * im.rows);

	//Scan de l'image
	int n= scanner.scan(image);

	//Impression des résultats
	for (Image::SymbolIterator symbol = image.symbol_begin(); symbol !=image.symbol_end(); ++symbol){
		decodedObject obj;
		obj.type= symbol->get_type_name();
		obj.data= symbol->get_data();

		//Impression du type et du data
		cout << "Type : " << obj.type <<endl;
		cout << "Data : " << obj.data <<endl<<endl;
		
		//Obtention de la localisation
		for(int i=0; i<symbol->get_location_size();i++){
			obj.location.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));

		}
		decodedObjects.push_back(obj);		 
	}

} 

//Apparition des QRCodes recherchés
void display(Mat &im, vector<decodedObject> &decodedObjects){
	
	//On boucle sur tous les objects décodés
	for(int i=0;i< decodedObjects.size();i++){
		vector<Point> points=decodedObjects[i].location;
		vector<Point> hull;

		// Si les points ne forment pas un carré on cherche une enveloppe convexe
		if(points.size()>4)
			convexHull(points, hull);
		else
			hull=points;

		int n=hull.size();

		for(int j=0;j<n;j++){
			line(im,hull[j],hull[(j+1) % n],Scalar(255,0,0),3);
		}


		int max=0;
		for(int i=0;i<n;i++){
			if(cv::norm(hull[i]-hull[(i+1) % n])>max)
				max=cv::norm(hull[i]-hull[(i+1) % n]);
		}
		cout << "Taille : " << max <<endl;
		
	}

	
	 // On affiche l'image dans une fenêtre
        imshow("Results",im);
        waitKey(5);
}


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
     capture = cvCreateCameraCapture(0);
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
	Mat im=cvarrToMat(image);	


	//Variable répertoriant les objects détectés
	vector<decodedObject> decodedObjects;

	//On trouve les QRCodes
	decode(im, decodedObjects);

	//On les affiches
	display(im, decodedObjects);

       

        ros::spinOnce();
     }

    // Properly stop the program:
    cvReleaseCapture(&capture);
    cvDestroyWindow("GeckoGeek Window");

    cout << "close CamConnection" << endl;

    return 0;
}

