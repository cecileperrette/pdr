#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

using namespace std;

// Elements déclarés comme globaux notamment utilisés pour les noeuds du robot :

tf::Vector3 _goal;
std::string _next_goal_frame_id, _goal_frame_id;
std::string _cmd_frame_id;

ros::Publisher _cmd_publisher;

tf::TransformListener * _listener;

double _dmax_l_speed, _max_a_speed;
float _robot_radius;

// Fonctions définies après le main :
void goal_subscriber(const geometry_msgs::PoseStamped & g);
void move();

int main(int argc, char **argv)
{
    // initialisation de ROS:
    cout << "Initialize QRMove::SimpleMove: " << endl;
    ros::init( argc, argv, "simplemove" );
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    _listener= new tf::TransformListener(node);

    // Configuration:
    float perception_distance;

    if( !node_private.getParam("robot_radius", _robot_radius) ) _robot_radius= 0.3f;
    if( !node_private.getParam("perception_distance", perception_distance) ) perception_distance= 2.0f;

    // Configuration des mouvememts :
    std::string goal_topic, cmd_topic;
    float goal_x, goal_y;

    if( !node_private.getParam("goal_topic", goal_topic) ) goal_topic= "move_base_simple/goal";
    if( !node_private.getParam("goal_frame_id", _next_goal_frame_id) ) _next_goal_frame_id= "odom";
    if( !node_private.getParam("init_goal_x", goal_x) ) goal_x= 0.f;
    if( !node_private.getParam("init_goal_y", goal_y) ) goal_y= 0.f;
    if( !node_private.getParam("init_goal_frame_id", _goal_frame_id) ) _goal_frame_id= "odom";

    if( !node_private.getParam("cmd_topic", cmd_topic) ) cmd_topic= "/cmd_vel_mux/input/navi";
    if( !node_private.getParam("cmd_frame_id", _cmd_frame_id) ) _cmd_frame_id= "base_link";

    if( !node_private.getParam("linear_speed", _dmax_l_speed) ) _dmax_l_speed= 0.2;
    if( !node_private.getParam("angular_speed", _max_a_speed) ) _max_a_speed= 1.2;

    _goal.setX( goal_x );
    _goal.setY( goal_y );
    _goal.setZ( 0.0f );

    // subscribers function:
    ros::Subscriber sub2 = node.subscribe( goal_topic, 1, goal_subscriber );

    // publisher function:
    _cmd_publisher= node.advertise<geometry_msgs::Twist>( cmd_topic, 1 );

    cout << "run SimpleMove" << endl;

//    ros::spin();
    ros::Rate loop_rate(10);
    while( ros::ok() )
    {

	// on appelle la fonction move avec les goals suivant x et y définis plus haut	
      move();
      ros::spinOnce();
      loop_rate.sleep();
    }

    // Properly stop the program:
    cout << "close SimpleMove" << endl;

    return 0;
}

// permet de prévoir les déplacements et d'appliquer les changements de goal
void goal_subscriber(const geometry_msgs::PoseStamped & g){

  cout << "goal_subscriber" << endl;

  _goal_frame_id= _next_goal_frame_id;

  _goal.setX( g.pose.position.x );
  _goal.setY( g.pose.position.y );
  _goal.setZ( g.pose.position.z );

  cout << "goal in " << g.header.frame_id  << " frame :" << _goal.x() << ", " << _goal.y() << endl;

  if ( _listener->waitForTransform( _goal_frame_id, g.header.frame_id, g.header.stamp, ros::Duration(0.5) ) )
  {
    tf::StampedTransform toRefGoalFrame;

    try{
      _listener->lookupTransform( _goal_frame_id, g.header.frame_id, g.header.stamp,  toRefGoalFrame );
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    _goal= toRefGoalFrame * _goal;

    cout << "goal in " << _goal_frame_id  << " frame :" << _goal.x() << ", " << _goal.y() << endl;
  }
  else{
    cerr << "Transform " <<  g.header.frame_id << "-" << _goal_frame_id << " unvailable" <<  endl;
  }
}


// fonction qui va faire bouger le robot
void move(){

  cout << "Move the robot" << endl;

  // On attend le Transform approprié :
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

  // Si transform existe on fait les changements appropriés:
  if( transform_ok )
  {
    localGoal= goalToCmd * _goal;

    cout << "\tlocal goal ("
      << localGoal.x() << ", " << localGoal.y()
      <<  ") in " << _cmd_frame_id << endl;

    // On affiche le mouvement prévu:

    cout << "\tmove to (" << _goal.x() << ", " << _goal.y()
      <<  ") in " << _goal_frame_id << " -> ("
      << localGoal.x() << ", " << localGoal.y()
      <<  ") in " << _cmd_frame_id << endl;
  }
  else{
    cout << "\tstop ! (" << _goal.x() << ", " << _goal.y() <<  ") -> (" << localGoal.x() << ", " << localGoal.y() <<  ")" << endl;
  }


  //On génère le message :
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

  cout << "-> generate the command: distance "
    << d  << " vs " << _robot_radius*0.2 << endl;

  if( d > _robot_radius*0.2 )// !stop condition :
  {
      cmd.linear.x= _dmax_l_speed + min( 0.5*localGoal.x()*_dmax_l_speed, _dmax_l_speed );

      if( 0.005f < norm_goal_y )
        cmd.angular.z= min( norm_goal_y*2.0*_max_a_speed, _max_a_speed );

      if( -0.005f > norm_goal_y )
        cmd.angular.z= -(min( norm_goal_y*-2.0*_max_a_speed, _max_a_speed ));
  }

  cout << "\tcommande linear: " << cmd.linear.x << ", angular: " << cmd.angular.z << endl;

  _cmd_publisher.publish( cmd );

  cout << "-> end generate the command: " << endl;
}
