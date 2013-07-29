//RVIZ measurement tool
//Qiaosong Wang
//University of Delaware
//qiaosong@udel.edu

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/bind.hpp>
#include <cmath>
#include <vector>
#include <math.h>

using namespace std;
//Defining ROS parameters
ros::Publisher marker_pub;
ros::NodeHandle *nhp;
ros::Subscriber click_sub;

float cpdist=0; //Current frame
float globalscale =0.05;
geometry_msgs::Point fp; //future point
geometry_msgs::Point cp; //current point
geometry_msgs::Point pp; //previous point

//Number of points, need to be dynamically assiagned once the panel is done
int num_polypoints=8;
int polycount = 0;

//Declare 3D point storage structure
vector<vector<float> > polypoints(3, vector<float>(100));

float calc_cp_dist(int i, vector<vector<float> > polypoints)
{
  float cp_dist= sqrt((polypoints[0][i]-polypoints[0][i-1])*(polypoints[0][i]-polypoints[0][i-1])+(polypoints[1][i]-polypoints[1][i-1])*(polypoints[1][i]-polypoints[1][i-1])+(polypoints[2][i]-polypoints[2][i-1])*(polypoints[2][i]-polypoints[2][i-1]));
  return cp_dist;
}

void clickCallback(const geometry_msgs::PointStamped& msg)
{

     //Getting ROS geometry msgs
    //ROS_INFO("New Point at X: %f, Y: %f, Z: %f\n", msg.point.x, msg.point.y, msg.point.z);
    fp.x =  msg.point.x;
    fp.y =  msg.point.y;
    fp.z =  msg.point.z;


    // Initialize marker parameters
    visualization_msgs::Marker points, line_strip, line_list , psphere, csphere, arrow, showtext,showtextid; 
    showtext.header.frame_id = arrow.header.frame_id=psphere.header.frame_id=csphere.header.frame_id = points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/base_link";
    showtext.header.stamp = arrow.header.stamp=psphere.header.stamp =csphere.header.stamp= points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    showtext.ns = arrow.ns=psphere.ns=csphere.ns=points.ns = line_strip.ns = line_list.ns = "UD_rviz_interaction";
    showtext.action = arrow.action=psphere.action=csphere.action=points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    showtext.pose.orientation.w = arrow.pose.orientation.w=psphere.pose.orientation.w =csphere.pose.orientation.w= points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    //ID
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
    psphere.id =3;
    csphere.id =4;
    arrow.id=5;
    showtext.id=6;
    showtextid.id=7;

    //Type
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    psphere.type = visualization_msgs::Marker::SPHERE;
    csphere.type = visualization_msgs::Marker::SPHERE;
    arrow.type = visualization_msgs::Marker::ARROW;
    showtext.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    showtextid.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    //Scale
    psphere.scale.x = globalscale;
    psphere.scale.y = globalscale;
    psphere.scale.z = globalscale;

    csphere.scale.x = globalscale;
    csphere.scale.y = globalscale;
    csphere.scale.z = globalscale;

    showtext.scale.x = 0.05;
    showtext.scale.y = 0.05;
    showtext.scale.z = 0.05;

    showtextid.scale.x = 0.05;
    showtextid.scale.y = 0.05;
    showtextid.scale.z = 0.05;
 
    line_list.scale.x = 0.005;
    line_list.scale.y = 0.005;
    line_list.scale.z = 0.005;

    line_strip.scale.x = 0.005;

    //Color
    points.color.b = 1.0;
    points.color.a = 1.0;

    // Line strip is blue
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;

     line_strip.color.b = 1.0;
     line_strip.color.a = 1.0;

    // Sphere is green
    psphere.color.r = 1.0;
    psphere.color.a = 0.6;

    csphere.color.g = 1.0;
    csphere.color.a = 1.0;

    //Text is green
    showtext.color.r = 1.0;
    showtext.color.g = 1.0;
    showtext.color.a = 1.0;

    showtextid.color.r = 0.0;
    showtextid.color.g = 1.0;
    showtextid.color.a = 1.0;

    cp.x=fp.x;
    cp.y=fp.y;
    cp.z=fp.z;

      psphere.points.push_back(pp);
      psphere.pose.position.x = pp.x;
      psphere.pose.position.y = pp.y;
      psphere.pose.position.z = pp.z;

      csphere.points.push_back(cp);
      csphere.pose.position.x = cp.x;
      csphere.pose.position.y = cp.y;
      csphere.pose.position.z = cp.z;





     // Create the vertices for the points and lines

    polypoints[0][polycount]=fp.x;
    polypoints[1][polycount]=fp.y;
    polypoints[2][polycount]=fp.z;


       for (int i = 0; i<=polycount;i++)
        {
          
           geometry_msgs::Point p;
           p.x = polypoints[0][i];
           p.y = polypoints[1][i];
	   p.z = polypoints[2][i];
           /*
           printf("%f\n", polypoints[0][i]);
           printf("%f\n", polypoints[1][i]);
           printf("%f\n", polypoints[2][i]);
           */
           points.points.push_back(p);
           line_strip.points.push_back(p);


            std::stringstream point_id;
            point_id<< i << "\n";
         
             printf("%d\n", i);
             
         showtextid.text = point_id.str();
         showtextid.pose.position.x = polypoints[0][i];
         showtextid.pose.position.y = polypoints[1][i];
         showtextid.pose.position.z = polypoints[2][i];

          // line_list.points.push_back(p);
          // p.z += 1.0;
           //line_list.points.push_back(p);
          if(i>0)
          cpdist=cpdist+calc_cp_dist(i,polypoints); 
        }

       pp.x = cp.x;
       pp.y = cp.y;
       pp.z = cp.z; 

         printf("Distance: %f \n", cpdist);

            std::stringstream show_dist;
            show_dist<< cpdist << "\n";

         showtext.text = show_dist.str();
         showtext.pose.position.x = polypoints[0][polycount];
         showtext.pose.position.y = polypoints[1][polycount];
         showtext.pose.position.z = polypoints[2][polycount];

// Publish markers
    marker_pub.publish(line_list);
    marker_pub.publish(psphere);
    marker_pub.publish(csphere);
    marker_pub.publish(showtext);
    marker_pub.publish(showtextid);
    marker_pub.publish(line_strip);


    if (polycount!=num_polypoints)
     polycount++;
    else
     {
     polycount=0;
     cpdist=0;
     }
  
}



int main( int argc, char** argv )
{

fp.x=1;
fp.y=1;
fp.z=1;
cp.x=pp.x=0;
cp.y=pp.y=0;
cp.z=pp.z=0;

  ros::init(argc, argv, "UD_rviz_interaction");

   nhp= new ros::NodeHandle();

  marker_pub = nhp->advertise<visualization_msgs::Marker>("visualization_marker", 10);
   click_sub = nhp->subscribe("clicked_point", 10,clickCallback);

  ros::Rate UD_rviz_interaction_rate(30);

  while (ros::ok())
  {
    UD_rviz_interaction_rate.sleep();
    ros::spinOnce();
  }
}


