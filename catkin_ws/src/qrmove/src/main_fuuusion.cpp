#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <stdio.h>
#include <iostream>     // std::cout
#include <cmath>        // std::abs
#include <stdlib.h>
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include "float2.h"

using namespace std;
using namespace cv;
using namespace zbar;

// On commence par définir la structure qui renfermera l'information concernant un QRCode détécté dans l'image donnée

typedef struct{
	string type;
	string data;
	vector <Point> location;
} decodedObject;


typedef enum{
	es_undef= 0,
	es_free,
	es_obst,
	es_qrc_far,
	es_qrc_near
} EState;
std::string _stateStr[]= {"undefined", "free", "obst", "qrc_far", "qrc_near"};

EState _state= es_undef;

// global element :

tf::Vector3 _goal;
std::string _next_goal_frame_id, _goal_frame_id;
std::string _cmd_frame_id;

ros::Publisher _cmd_publisher;

tf::TransformListener * _listener;

double _dmax_l_speed, _max_a_speed;
float _robot_radius;
float taille=0;
std::vector<mia::Float2> _scan;

// Fonctions
void decode(Mat &im, vector<decodedObject> &decodedObjects);
void display(Mat &im, vector<decodedObject> &decodedObjects);
void goal_subscriber(const geometry_msgs::PoseStamped & g);
void record_goal(const tf::Vector3 & g, std::string frame_id);
void move();
void scan_subscriber( const sensor_msgs::LaserScan& scan );

int main(int argc, char **argv)
{
    // ROS:
    cout << "Initialize QRMove::CamConnection: " << endl;
    ros::init( argc, argv, "camconnection" );
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

_listener= new tf::TransformListener(node);

    // Configuration:
    float perception_distance;

    if( !node_private.getParam("robot_radius", _robot_radius) ) _robot_radius= 0.3f;
    if( !node_private.getParam("perception_distance", perception_distance) ) perception_distance= 2.0f;

    // Configuration Movement:
    std::string goal_topic, cmd_topic;
    float goal_x, goal_y;

    if( !node_private.getParam("goal_topic", goal_topic) ) goal_topic= "move_base_simple/goal";
    if( !node_private.getParam("goal_frame_id", _next_goal_frame_id) ) _next_goal_frame_id= "odom";
    if( !node_private.getParam("init_goal_x", goal_x) ) goal_x= 0.f;
    if( !node_private.getParam("init_goal_y", goal_y) ) goal_y= 0.f;
    if( !node_private.getParam("init_goal_frame_id", _goal_frame_id) ) _goal_frame_id= "odom";

    if( !node_private.getParam("cmd_topic", cmd_topic) ) cmd_topic= "/cmd_vel_mux/input/navi";
    if( !node_private.getParam("cmd_frame_id", _cmd_frame_id) ) _cmd_frame_id= "base_link";

    if( !node_private.getParam("linear_speed", _dmax_l_speed) ) _dmax_l_speed= 0.06f;
    if( !node_private.getParam("angular_speed", _max_a_speed) ) _max_a_speed= 0.6f;

    _goal.setX( goal_x );
    _goal.setY( goal_y );
    _goal.setZ( 0.0f );

    // subscribers function:
    ros::Subscriber sub2 = node.subscribe( goal_topic, 1, goal_subscriber );
	ros::Subscriber sub3 = node.subscribe( "scan", 1, scan_subscriber );

    // publisher function:
    _cmd_publisher= node.advertise<geometry_msgs::Twist>( cmd_topic, 1 );

    // get the hand to ros:
    cout << "run fuuusion" << endl;


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


     ros::Rate loop_rate(10);
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

	bool obstacle=false;
	for(int i=0;i<_scan.size();i++){
		if (_scan[i].x<0.1 || _scan[i].y<0.1){
			obstacle=true;
		}
	}


	//On envoit les nouveaux goals:
	EState last_state= _state;	
	if (taille==0 && !obstacle){
		_state= es_free;
		goal_x=2.f;
		goal_y=-0.5f;
	}
	else if(taille!=0 && taille<250){
		_state= es_qrc_far;
			goal_x=0.2f;
			goal_y=0.f;	
	}
	else if(taille>=250){
		_state= es_qrc_near;
		cout << "SWITCH FROOOOOM " << _stateStr[last_state].c_str() << " TOOOO DODO" << endl;
		break;
	}
	else if (obstacle) {
		_state= es_obst;
		goal_x=-0.5;
		goal_y=0;
	}

	if( last_state != _state ){
		cout << "SWITCH FROOOOOM " << _stateStr[last_state].c_str() << " TOOOO " << _stateStr[_state].c_str() << endl;
	}

	_goal.setX( goal_x );
    	_goal.setY( goal_y );
	record_goal( _goal, "base_link");

	move();
	ros::spinOnce();
	
     }

    // Properly stop the program:
    cvReleaseCapture(&capture);
    cvDestroyWindow("GeckoGeek Window");

    cout << "close Fuuusion" << endl;

    return 0;
}


// on écrit une fonction qui va repérer des QRCode
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
		cout << "Information : " << obj.data <<endl<<endl;
		
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
		taille=max;
		cout << "Taille : " << taille <<endl;
		
	}

	
	 // On affiche l'image dans une fenêtre
        imshow("Results",im);
        waitKey(5);
}

void goal_subscriber(const geometry_msgs::PoseStamped & g){
  //cout << "goal_subscriber" << endl;
  _goal_frame_id= _next_goal_frame_id;

  _goal.setX( g.pose.position.x );
  _goal.setY( g.pose.position.y );
  _goal.setZ( g.pose.position.z );

  record_goal( _goal,  g.header.frame_id );
}

void record_goal(const tf::Vector3 & g, std::string frame_id){

  //cout << "goal in " << g.header.frame_id  << " frame :" << _goal.x() << ", " << _goal.y() << endl;
  //ros::Time now = g.header.stamp;
  ros::Time now = ros::Time::now();

  if ( _listener->waitForTransform( _goal_frame_id, frame_id, now, ros::Duration(0.5) ) )
  {
    tf::StampedTransform toRefGoalFrame;

    try{
      _listener->lookupTransform( _goal_frame_id, frame_id, now,  toRefGoalFrame );
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    _goal= toRefGoalFrame * _goal;

    //cout << "goal in " << _goal_frame_id  << " frame :" << _goal.x() << ", " << _goal.y() << endl;
  }
  else{
    //cerr << "Transform " <<  g.header.frame_id << "-" << _goal_frame_id << " unvailable" <<  endl;
  }
}

void move(){

  //cout << "Move the robot" << endl;

  // Wait for appropriate Transform :
  if ( _goal_frame_id.compare( _cmd_frame_id ) != 0
     && _cmd_frame_id.compare( _goal_frame_id ) != 0
     && !_listener->waitForTransform( _cmd_frame_id, _goal_frame_id,
                                      ros::Time(0), ros::Duration(0.5) )
  )
    cerr << "Command transform: " << _goal_frame_id
        << " -> " << _cmd_frame_id << " unvailable." << endl;

  bool transform_ok= true;

  // Get Transforms (goal -> scan frames and scan -> commande frames):

  tf::StampedTransform goalToCmd;
  try{
    _listener->lookupTransform( _cmd_frame_id, _goal_frame_id,
                                ros::Time(0), goalToCmd );
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    transform_ok= false;
  }

  tf::Vector3 localGoal(0.f, 0.f, 0.f);

  // If transforms exist move goal in scan frame:
  if( transform_ok )
  {
    localGoal= goalToCmd * _goal;

    /*cout << "\tlocal goal ("
      << localGoal.x() << ", " << localGoal.y()
      <<  ") in " << _cmd_frame_id << endl;*/

    // generate appropriate commande message :

    /*cout << "\tmove to (" << _goal.x() << ", " << _goal.y()
      <<  ") in " << _goal_frame_id << " -> ("
      << localGoal.x() << ", " << localGoal.y()
      <<  ") in " << _cmd_frame_id << endl;*/
  }
  else{
    //cout << "\tstop ! (" << _goal.x() << ", " << _goal.y() <<  ") -> (" << localGoal.x() << ", " << localGoal.y() <<  ")" << endl;
  }


  // generate appropriate commande message :
  float d= sqrtf( (localGoal.x()*localGoal.x()
                  + localGoal.y()*localGoal.y()) );
  float norm_goal_y= localGoal.y() / d;

  geometry_msgs::Twist cmd;

  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  /*cout << "-> generate the command: distance "
    << d  << " vs " << _robot_radius*0.2 << endl;*/

  if( d > _robot_radius*0.2 )// !stop condition :
  {
      cmd.linear.x= _dmax_l_speed + min( 0.5*localGoal.x()*_dmax_l_speed, _dmax_l_speed );

      if( 0.005f < norm_goal_y )
        cmd.angular.z= min( norm_goal_y*2.0*_max_a_speed, _max_a_speed );

      if( -0.005f > norm_goal_y )
        cmd.angular.z= -(min( norm_goal_y*-2.0*_max_a_speed, _max_a_speed ));
  }

  //cout << "\tcommande linear: " << cmd.linear.x << ", angular: " << cmd.angular.z << endl;

  _cmd_publisher.publish( cmd );

  //cout << "-> end generate the command: " << endl;
}


void scan_subscriber( const sensor_msgs::LaserScan& scan ){
  std::vector<mia::Float2> lscan;
  int size= scan.ranges.size();
  float angle( scan.angle_min );

  for(int i(0); i < size; ++i )
  {
      if( 0.3 < scan.ranges[i] && scan.ranges[i] < 2.0 )
      {
        lscan.push_back( mia::Float2::direction( angle ) * scan.ranges[i] );
      }
      angle+= scan.angle_increment;
  }

  _scan.reserve( lscan.size() );
  std::copy(std::begin(lscan), std::end(lscan), std::back_inserter(_scan));
  _scan= lscan;
}



