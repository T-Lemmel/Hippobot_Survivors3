#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include<algorithm>
#include<vector>
#include <astar_path_planner/a_star.h>
#include <astar_path_planner/point.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>


using namespace std::chrono_literals;
using namespace std;
using namespace ecn;

class path_planner_cell_node : public rclcpp::Node
{
public:
    path_planner_cell_node() : Node("path_planner_cell_node")
    {

        // Log that the node has succesfully started
        RCLCPP_INFO(this->get_logger(), "path planner cell has successfully started");


        // Create a subscrber that get information to update the Map
        map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>("mapping", 10,std::bind(&path_planner_cell_node::updateMap, this, std::placeholders::_1));

        // Create subscribers that will receive a geometry_msgs::msgs::Pose message
        //position of the ship
        pos_sub = create_subscription<geometry_msgs::msg::Pose>("position", 10, std::bind(&path_planner_cell_node::positionCallback, this, std::placeholders::_1));
        //position of the target
        targ_pos_sub = create_subscription<geometry_msgs::msg::Pose>("target_position", 10, std::bind(&path_planner_cell_node::targetCallback, this, std::placeholders::_1));

        // Create a timer that will call the timer_callback function
        timer_ = this->create_wall_timer(500ms, std::bind(&path_planner_cell_node::timer_callback, this));

        // Create a publisher on the topic ??? for the PID to get the next waypoint
        path_pub = create_publisher<nav_msgs::msg::Path>("waypoint", 10);

        //initialise pointZero orientation, mais est-ce vraiment utile?
        pointZero.orientation.set__w(0);
        pointZero.orientation.set__x(0);
        pointZero.orientation.set__y(0);
        pointZero.orientation.set__z(0);

        //Pour voir la map dans Rviz
        set_parameter(rclcpp::Parameter("use_sim_time",true));
        map.header.frame_id = "map";
        Path.header.frame_id = "map";


    };


    void updateMap(const nav_msgs::msg::OccupancyGrid &NewMap)
    {
        map.data = NewMap.data;
        map.info = NewMap.info;
        map_init = true;
        if(map_init && targ_init && my_posinit) initialisation_done = true;
    }


    void positionCallback(const geometry_msgs::msg::Pose &msg)
    {
        my_position.x_ = msg.position.x;
        my_position.y_ = msg.position.y;
        my_posinit = true;
        if(map_init && targ_init && my_posinit) initialisation_done = true;
    }

    void targetCallback(const geometry_msgs::msg::Pose &msg)
    {
        target_position.x_ = msg.position.x;
        target_position.y_ = msg.position.y;
        targ_init = true;
        if(map_init && targ_init && my_posinit) initialisation_done = true;
    }

    void timer_callback()
    {
        if(initialisation_done)
        {
            my_position.map_ = map;
            target_position.map_ = map;
            point_Path = Astar(my_position,target_position);
            path_size = point_Path.size();
            if(path_size>0)
            {
                Path.poses.resize(path_size);
                for(int index=0;index<path_size;index++)
                {
                    //On prend la valeur du point x et en y à l'indice index du point_Path pour le stocker dans le Path que l'on envoie
                    Path.poses[index].pose.position.set__x(point_Path[index].x_);
                    Path.poses[index].pose.position.set__y((point_Path[index].y_));
                    Path.poses[index].pose.position.set__z(point_Path[index].map_.data[point_Path[index].y_*point_Path[index].map_.info.width+point_Path[index].x_]);
                    Path.poses[index].pose.set__orientation(pointZero.orientation);//utile de mettre les orientations à 0 à chaque fois?

                    cout<<"Path changed "<<index<<" times"<<endl;
                }
                path_pub->publish(Path);
            }
        }

    }



    //Talk and subscribe
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pos_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr targ_pos_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    //Variables
    nav_msgs::msg::Path Path;
    vector<Point> point_Path;
    int path_size;
    nav_msgs::msg::OccupancyGrid map;
    Point my_position;
    Point target_position;
    geometry_msgs::msg::Pose pointZero;
    bool initialisation_done=false;
    bool map_init=false;
    bool targ_init=false;
    bool my_posinit=false;



};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_planner_cell_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

