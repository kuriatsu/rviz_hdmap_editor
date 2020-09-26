#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <fstream>
#include <vector>
//
// #include <pcl_ros/point_cloud.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

class AdasVis
{
private:
	ros::Publisher pub_marker;
    // ros::Publisher pub_img_points;

    ros::Timer timer;

	std::map<int ,std::vector<std::string>> adas_points;
	std::map<int, std::vector<std::string>> adas_lines;
	std::map<int, std::vector<std::string>> adas_whitelines;
	std::string m_adas_points_header;
    // cv::Mat m_map_image;
    // geometry_msgs::Point m_map_origin;
    // float m_map_res;

public:
	AdasVis();
	~AdasVis();

private:
    // void timerCallback(const ros::TimerEvent &);
	std::map<int, std::vector<std::string>> readCsv(const std::string &filename);
	visualization_msgs::InteractiveMarker makeIntMarker(const std::string &name, const geometry_msgs::Point &point);
	void intMarkerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    // void makeImagePoints();
	void refleshAdasMarker();

};

AdasVis::AdasVis()
{
	ros::NodeHandle n;
	server.reset(new interactive_markers::InteractiveMarkerServer("adas_vis_node"));
	pub_marker = n.advertise<visualization_msgs::MarkerArray>("/adas_vis_marker", 5);
    // pub_img_points = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/image_points", 5);

	adas_lines = readCsv("/media/kuriatsu/SamsungKURI/brainIV/map/mfds_adas_new/line.csv");
	adas_whitelines = readCsv("/media/kuriatsu/SamsungKURI/brainIV/map/mfds_adas_new/whiteline.csv");
	adas_points = readCsv("/media/kuriatsu/SamsungKURI/brainIV/map/mfds_adas_new/point.csv");
    //
    // m_map_image = cv::imread("/home/kuriatsu/Pictures/mfds_satelite.png", 1);
    // m_map_origin.x = 0;
    // m_map_origin.y = 0;
    // m_map_origin.z = 0;
    // m_map_res = 1.0;
    // makeImagePoints();
	refleshAdasMarker();
    //
    // ros::Duration(1).sleep();
    // timer = n.createTimer(ros::Duration(100), &AdasVis::timerCallback, this);

}


AdasVis::~AdasVis()
{
	server.reset();
	std::ofstream ofs("/media/kuriatsu/SamsungKURI/brainIV/map/mfds_points_new.csv");
	ofs << m_adas_points_header << std::endl;

	for(auto &e : adas_points)
	{
		for (int j = 0; j < e.second.size() - 1 ; j++)
		{
			ofs << e.second[j] << ",";
		}
		ofs << e.second.back() << std::endl;
	}
}

//
// void AdasVis::timerCallback(const ros::TimerEvent&)
// {
//     makeImagePoints();
// }

// void AdasVis::makeImagePoints()
// {
//     pcl::PointCloud<pcl::PointXYZRGB> image_points;
//     int height = m_map_image.rows;
//     int width = m_map_image.cols;
//     std::cout << width << std::endl;
//     for (int i = 0; i < height; i++)
//     {
//         cv::Vec3b *col = m_map_image.ptr<cv::Vec3b>(i);
//         for (int j=0; j<width; j++)
//         {
//             pcl::PointXYZRGB point;
//             point.x = i * m_map_res + m_map_origin.x;
//             point.y = j * m_map_res + m_map_origin.y;
//             // std::cout << point.x << "," << point.y << std::endl;
//             point.z = 0;
//             point.r = col[j][2];
//             point.g = col[j][1];
//             point.b = col[j][0];
//             // std::cout << point << std::endl;
//             image_points.push_back(point);
//         }
//     }
//     std::cout << "pub" << std::endl;
//
//     auto msg = image_points.makeShared();
//     msg->header.frame_id = "map";
//     pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
//
//     pub_img_points.publish(msg);
//
// }

std::map<int, std::vector<std::string>> AdasVis::readCsv(const std::string &filename)
{
	std::ifstream ifs(filename);
	std::string line, field;
	std::vector<std::string> result_row;
	std::map<int, std::vector<std::string>> result_out;

	std::getline(ifs, m_adas_points_header);

	while(std::getline(ifs, line))
	{
		std::istringstream stream(line);
		while(std::getline(stream, field, ','))
		{
			result_row.emplace_back(field);
		}
        // if (result_out.count(std::stoi(result_row[0])) != 0)
        // {
        //     std::cout << result_row[0] << std::endl;
        // }
		result_out[std::stoi(result_row[0])] = result_row;
		result_row.clear();
	}
	// std::cout << result_out.size() << std::endl;
	return result_out;
}


void AdasVis::refleshAdasMarker()
{
	std::stringstream ss;
	int bpid, fpid;
	geometry_msgs::Point p;
	std::vector<std::string> point;
    std::vector<int> point_names;

	for(const auto &e : adas_whitelines)
	{
		for (int i = 1; i <= 2; i++)
		{
            int name = std::stoi(adas_lines.at(std::stoi(e.second[1]))[i]);
			point = adas_points.at(std::stoi(adas_lines.at(std::stoi(e.second[1]))[i]));
            if (std::find(point_names.begin(), point_names.end(), name) != point_names.end())
            {
                continue;
            }
            point_names.emplace_back(std::stoi(adas_lines.at(std::stoi(e.second[1]))[i]));
			// p.x = std::stof(point[4]) - std::stof(adas_points.at(1)[4]);
			// p.y = std::stof(point[5]) - std::stof(adas_points.at(1)[5]);

            p.y = std::stof(point[4]);
			p.x = std::stof(point[5]);
			p.z = std::stof(point[3]);
			server->insert(makeIntMarker(point[0], p));
			server->setCallback(point[0], boost::bind(&AdasVis::intMarkerCb, this, _1));
		}
	}
	server->applyChanges();

}


visualization_msgs::InteractiveMarker AdasVis::makeIntMarker(const std::string &name, const geometry_msgs::Point &point)
{

	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = "map";
	int_marker.name = name;
	int_marker.scale = 0.1;
	int_marker.pose.position = point;
	int_marker.pose.orientation.x = 0.0;
	int_marker.pose.orientation.y = 0.0;
	int_marker.pose.orientation.z = 0.0;
	int_marker.pose.orientation.w = 1.0;



	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible = true;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.orientation.w = 1;

	visualization_msgs::Marker marker;
	marker.ns="point";
	marker.id = std::stoi(name);
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	marker.color.r = 1;
	marker.color.g = 1;
	marker.color.b = 1;
	marker.color.a = 0.5;

	control.markers.emplace_back(marker);
    int_marker.controls.emplace_back(control);

    // control.always_visible = true;
    // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    // control.orientation.x = 0;
    // control.orientation.y = 1;
    // control.orientation.z = 0;
    // control.orientation.w = 1;

    // marker.ns="range";
	// marker.id = std::stoi(name);
	// marker.type = visualization_msgs::Marker::CYLINDER;
	// marker.action = visualization_msgs::Marker::ADD;
	// marker.scale.x = 3.5;
	// marker.scale.y = 3.5;
	// marker.scale.z = 0.1;
	// marker.color.r = 0.5;
	// marker.color.g = 0.5;
	// marker.color.b = 1;
	// marker.color.a = 0.2;

    // control.markers.emplace_back(marker);
	// int_marker.controls.emplace_back(control);
	return int_marker;
}


void AdasVis::intMarkerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	adas_points.at(std::stoi(feedback->marker_name))[3] = std::to_string(feedback->pose.position.z);
	adas_points.at(std::stoi(feedback->marker_name))[4] = std::to_string(feedback->pose.position.y);
	adas_points.at(std::stoi(feedback->marker_name))[5] = std::to_string(feedback->pose.position.x);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "adas_vis_node");
	AdasVis adas_vis;
	ros::spin();
	return 0;
}
