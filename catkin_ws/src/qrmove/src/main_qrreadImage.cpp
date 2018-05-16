#include <opencv2/opencv.hpp>
#include <zbar.h>


using namespace cv;
using namespace std;
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

		// Formation d'une enveloppe convexe
		if(points.size()>4)
			convexHull(points, hull);
		else
			hull=points;

		int n=hull.size();

		for(int j=0;j<n;j++){
			line(im,hull[j],hull[(j+1) % n],Scalar(255,0,0),3);
		}
	}
	
	//On display le tout
	imshow("Results",im);
	waitKey(0);
}

int main(int argc, char* argv[]){

	//On lit l'image
	Mat im= imread("index.png");

	//Variable répertoriant les objects détectés
	vector<decodedObject> decodedObjects;

	//On trouve les QRCodes
	decode(im, decodedObjects);

	//On les affiches
	display(im, decodedObjects);

	return EXIT_SUCCESS;

}


