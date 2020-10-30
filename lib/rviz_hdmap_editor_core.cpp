#include "rviz_hdmap_editor_core.h"
#include <ros/ros.h>

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> int_marker_server;

RvizHdmapEditorCore::RvizHdmapEditorCore()
{
	ros::NodeHandle n;
	int_marker_server.reset(new interactive_markers::InteractiveMarkerServer("adas_vis_node"));
	pub_marker = n.advertise<visualization_msgs::MarkerArray>("/adas_vis_marker", 5);

}


RvizHdmapEditorCore::~RvizHdmapEditorCore()
{
	int_marker_server.reset();
}


void RvizHdmapEditorCore::readHdmap(const std::string &filename)
{
	if (filename.empty()) return;

	std::ifstream ifs(filename);
	std::string line, field, header, type;
	std::vector<std::string> raw_data_list;
	std::map<std::string, std::vector<std::string>> extracted_data;
	HdmapInfo hdmap_info;

	hdmap_info.input_filename = filename;
	hdmap_info.is_visible = false;
	std::getline(ifs, hdmap_info.header);
	ROS_INFO_STREAM(filename);

	while(std::getline(ifs, line))
	{
		std::istringstream stream(line);
		while(std::getline(stream, field, ','))
		{
			raw_data_list.emplace_back(field);
		}
		extracted_data[raw_data_list[0]] = raw_data_list;
		raw_data_list.clear();
	}	
	std::cout << "got data " << std::endl;


	hdmap_info.data = extracted_data;
	hdmap_info_dict[getHdmapType(filename)] = hdmap_info;


}

std::string RvizHdmapEditorCore::getHdmapType(const std::string &filename)
{
	std::string name = filename.substr(filename.find_last_of("/")+1);
	std::string type = name.substr(0, name.find_last_of("."));
	// ROS_INFO_STREAM(name);
	std::cout << type << std::endl;
	return type;
}


void RvizHdmapEditorCore::saveHdmap(const std::string &dirname)
{
	for (auto &hdmap_elem : hdmap_info_dict)
	{
		std::string filename;
		filename = dirname + "/" + hdmap_elem.first + ".csv";
		std::cout << filename << std::endl;
		std::ofstream ofs(filename);

		ofs << hdmap_elem.second.header << std::endl;

		for(auto &data_elem: hdmap_elem.second.data)
		{
			for (int j = 0; j < data_elem.second.size() - 1 ; j++)
			{
				ofs << data_elem.second[j] << ",";
			}
			ofs << data_elem.second.back() << std::endl;
		}
	}
}


void RvizHdmapEditorCore::toggleHdmapElement(const std::string &element_name, const bool &toggle)
{
	std::cout << "toggle " << std::endl;

	if (hdmap_info_dict.count(element_name))
	{
		hdmap_info_dict.at(element_name).is_visible = toggle;
	}

	refleshAdasMarker();
}


void RvizHdmapEditorCore::refleshAdasMarker()
{
	makeWhiteline();
	makeRoadedge();
	makeStopline();
	makeRailroad();
	makeCrosswalk();
	makeIntersection();
	makePole();
	makeSignal();
	makeLane();
	makeNode();
	int_marker_server->applyChanges();

}


void RvizHdmapEditorCore::makeWhiteline()
{
	if (!hdmap_info_dict.at("whiteline").is_visible) return;

	std::cout << "make whiteline" << std::endl; 
	geometry_msgs::Point p;
    std::vector<std::string> added_points;
    auto &whiteline_data = hdmap_info_dict.at("whiteline").data;
    auto &line_data = hdmap_info_dict.at("line").data;
    auto &point_data = hdmap_info_dict.at("point").data;

	std_msgs::ColorRGBA color;
	color.r = 1.0;
	color.g = 1.0;
	color.b = 1.0;
	color.a = 1.0;

	for (const auto &whiteline_elem : whiteline_data)
	{
		for (int i = 1; i <= 2; i++)
		{
			std::string point_id = line_data.at(whiteline_elem.second[1])[i];

            if (std::find(added_points.begin(), added_points.end(), point_id) != added_points.end())
            {
                continue;
            }

            added_points.emplace_back(point_id);
			std::vector<std::string> &point = point_data.at(point_id);

            p.y = std::stof(point[4]);
			p.x = std::stof(point[5]);
			p.z = std::stof(point[3]);

			int_marker_server->insert(makeIntPoint(point_id, p, color));
			int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intMarkerCb, this, _1));
		}
	}
}

void RvizHdmapEditorCore::makeRoadedge()
{
	if (!hdmap_info_dict.at("roadedge").is_visible) return;
	std::cout << "make roadedge" << std::endl; 

	geometry_msgs::Point p;
    std::vector<std::string> added_points;
    auto &roadedge_data = hdmap_info_dict.at("roadedge").data;
    auto &line_data = hdmap_info_dict.at("line").data;
    auto &point_data = hdmap_info_dict.at("point").data;

	std_msgs::ColorRGBA color;
	color.r = 0.2;
	color.g = 0.2;
	color.b = 0.2;
	color.a = 1.0;

	for (const auto &roadedge_elem : roadedge_data)
	{
		for (int i = 1; i <= 2; i++)
		{
			std::string point_id = line_data.at(roadedge_elem.second[1])[i];

            if (std::find(added_points.begin(), added_points.end(), point_id) != added_points.end())
            {
                continue;
            }

            added_points.emplace_back(point_id);
			std::vector<std::string> &point = point_data.at(point_id);

            p.y = std::stof(point[4]);
			p.x = std::stof(point[5]);
			p.z = std::stof(point[3]);

			int_marker_server->insert(makeIntPoint(point_id, p, color));
			int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intMarkerCb, this, _1));
		}
	}
}

void RvizHdmapEditorCore::makeStopline()
{
	if (!hdmap_info_dict.at("stopline").is_visible) return;
	std::cout << "make stopline" << std::endl; 

	geometry_msgs::Point p;
    std::vector<std::string> added_points;
    auto &stopline_data = hdmap_info_dict.at("stopline").data;
    auto &line_data = hdmap_info_dict.at("line").data;
    auto &point_data = hdmap_info_dict.at("point").data;

	std_msgs::ColorRGBA color;
	color.r = 1.0;
	color.g = 0.0;
	color.b = 0.0;
	color.a = 1.0;

	for (const auto &stopline_elem : stopline_data)
	{
		for (int i = 1; i <= 2; i++)
		{
			std::string point_id = line_data.at(stopline_elem.second[1])[i];

            if (std::find(added_points.begin(), added_points.end(), point_id) != added_points.end())
            {
                continue;
            }

            added_points.emplace_back(point_id);
			std::vector<std::string> &point = point_data.at(point_id);

            p.y = std::stof(point[4]);
			p.x = std::stof(point[5]);
			p.z = std::stof(point[3]);

			int_marker_server->insert(makeIntPoint(point_id, p, color));
			int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intMarkerCb, this, _1));
		}
	}
}


void RvizHdmapEditorCore::makeNode()
{
	if (!hdmap_info_dict.at("node").is_visible) return;
	std::cout << "make node" << std::endl; 

	geometry_msgs::Point p;
    std::vector<std::string> added_points;
    auto &node_data = hdmap_info_dict.at("node").data;
    auto &point_data = hdmap_info_dict.at("point").data;
	std::cout << "extract node" << std::endl; 

	std_msgs::ColorRGBA color;
	color.r = 0.0;
	color.g = 1.0;
	color.b = 0.0;
	color.a = 1.0;

	for (const auto &node_elem : node_data)
	{
		std::string point_id = node_elem.second[1];

        if (std::find(added_points.begin(), added_points.end(), point_id) != added_points.end())
        {
            continue;
        }

        try
        {
			std::vector<std::string> &point = point_data.at(point_id);
			added_points.emplace_back(point_id);

	        p.y = std::stof(point[4]);
			p.x = std::stof(point[5]);
			p.z = std::stof(point[3]);
			// std::cout << node_elem.second[0] << std::endl;

			int_marker_server->insert(makeIntPoint(point_id, p, color));
			int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intMarkerCb, this, _1));
        }
        catch (std::out_of_range e)
		{
			std::cout << "no point : " << point_id << std::endl;
		}


	}
}

void RvizHdmapEditorCore::makeRailroad()
{
	if (!hdmap_info_dict.at("railroad_crossing").is_visible) return;
	std::cout << "make railroad_crossing" << std::endl; 

	geometry_msgs::Point p;
    std::vector<std::string> added_points;
    auto &railroad_data = hdmap_info_dict.at("railroad_crossing").data;
    auto &area_data = hdmap_info_dict.at("area").data;
    auto &line_data = hdmap_info_dict.at("line").data;
    auto &point_data = hdmap_info_dict.at("point").data;

	std_msgs::ColorRGBA color;
	color.r = 0.0;
	color.g = 1.0;
	color.b = 1.0;
	color.a = 1.0;

	for (const auto &railroad_elem : railroad_data)
	{
		for (int line_id = std::stoi(area_data.at(railroad_elem.second[1])[1]); line_id <= stoi(area_data.at(railroad_elem.second[1])[2]); ++line_id)
		{
			for (int i=1; i<2; ++i)
			{
				std::string point_id = line_data.at(std::to_string(line_id))[i];
		        if (std::find(added_points.begin(), added_points.end(), point_id) != added_points.end())
		        {
		            continue;
		        }
		        added_points.emplace_back(point_id);
				std::vector<std::string> &point = point_data.at(point_id);

		        p.y = std::stof(point[4]);
				p.x = std::stof(point[5]);
				p.z = std::stof(point[3]);

				int_marker_server->insert(makeIntPoint(point_id, p, color));
				int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intMarkerCb, this, _1));
			}
		}
	}
}


void RvizHdmapEditorCore::makeCrosswalk()
{
	if (!hdmap_info_dict.at("crosswalk").is_visible) return;
	std::cout << "make crosswalk" << std::endl; 

	geometry_msgs::Point p;
    std::vector<std::string> added_points;
    auto &crosswalk_data = hdmap_info_dict.at("crosswalk").data;
    auto &area_data = hdmap_info_dict.at("area").data;
    auto &line_data = hdmap_info_dict.at("line").data;
    auto &point_data = hdmap_info_dict.at("point").data;

	std_msgs::ColorRGBA color;
	color.r = 1.0;
	color.g = 1.0;
	color.b = 0.0;
	color.a = 1.0;

	for (const auto &crosswalk_elem : crosswalk_data)
	{
		for (int line_id = std::stoi(area_data.at(crosswalk_elem.second[1])[1]); line_id <= std::stoi(area_data.at(crosswalk_elem.second[1])[2]); ++line_id)
		{
			for (int i=1; i<2; ++i)
			{
				std::string point_id = line_data.at(std::to_string(line_id))[i];
		        if (std::find(added_points.begin(), added_points.end(), point_id) != added_points.end())
		        {
		            continue;
		        }
		        added_points.emplace_back(point_id);
				std::vector<std::string> &point = point_data.at(point_id);

		        p.y = std::stof(point[4]);
				p.x = std::stof(point[5]);
				p.z = std::stof(point[3]);

				int_marker_server->insert(makeIntPoint(point_id, p, color));
				int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intMarkerCb, this, _1));
			}
		}
	}
}


void RvizHdmapEditorCore::makeIntersection()
{
	if (!hdmap_info_dict.at("intersection").is_visible) return;
	std::cout << "make intersection" << std::endl; 

	geometry_msgs::Point p;
    std::vector<std::string> added_points;
    auto &intersection_data = hdmap_info_dict.at("intersection").data;
    auto &area_data = hdmap_info_dict.at("area").data;
    auto &line_data = hdmap_info_dict.at("line").data;
    auto &point_data = hdmap_info_dict.at("point").data;

	std_msgs::ColorRGBA color;
	color.r = 0.0;
	color.g = 0.5;
	color.b = 0.5;
	color.a = 1.0;

	for (const auto &intersection_elem : intersection_data)
	{
		for (int line_id = std::stoi(area_data.at(intersection_elem.second[1])[1]); line_id <= std::stoi(area_data.at(intersection_elem.second[1])[2]); ++line_id)
		{
			for (int i=1; i<2; ++i)
			{
				std::string point_id = line_data.at(std::to_string(line_id))[i];
		        if (std::find(added_points.begin(), added_points.end(), point_id) != added_points.end())
		        {
		            continue;
		        }
		        added_points.emplace_back(point_id);
				std::vector<std::string> &point = point_data.at(point_id);

		        p.y = std::stof(point[4]);
				p.x = std::stof(point[5]);
				p.z = std::stof(point[3]);

				int_marker_server->insert(makeIntPoint(point_id, p, color));
				int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intMarkerCb, this, _1));
			}
		}
	}
}


void RvizHdmapEditorCore::makePole()
{
	if (!hdmap_info_dict.at("pole").is_visible) return;
	std::cout << "make pole" << std::endl; 

	geometry_msgs::Point p;
    std::vector<std::string> added_points;
    auto &pole_data = hdmap_info_dict.at("pole").data;
    auto &vector_data = hdmap_info_dict.at("vector").data;
    auto &point_data = hdmap_info_dict.at("point").data;

	std_msgs::ColorRGBA color;
	color.r = 0.5;
	color.g = 0.5;
	color.b = 0.0;
	color.a = 1.0;

	for (const auto &pole_elem : pole_data)
	{

		std::string point_id = vector_data.at(pole_elem.second[1])[1];
        if (std::find(added_points.begin(), added_points.end(), point_id) != added_points.end())
        {
            continue;
        }
        added_points.emplace_back(point_id);
		std::vector<std::string> &point = point_data.at(point_id);

        p.y = std::stof(point[4]);
		p.x = std::stof(point[5]);
		p.z = std::stof(point[3]);

		int_marker_server->insert(makeIntPoint(point_id, p, color));
		int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intMarkerCb, this, _1));
	}
}

void RvizHdmapEditorCore::makeSignal()
{
	if (!hdmap_info_dict.at("signaldata").is_visible) return;
	std::cout << "make signaldata" << std::endl; 

	geometry_msgs::Point p;
    std::vector<std::string> added_points;
    auto &signal_data = hdmap_info_dict.at("signaldata").data;
    auto &vector_data = hdmap_info_dict.at("vector").data;
    auto &point_data = hdmap_info_dict.at("point").data;

	std_msgs::ColorRGBA color;
	color.r = 0.0;
	color.g = 0.5;
	color.b = 1.0;
	color.a = 1.0;

	for (const auto &signal_elem : signal_data)
	{

		std::string point_id = vector_data.at(signal_elem.second[1])[1];
        if (std::find(added_points.begin(), added_points.end(), point_id) != added_points.end())
        {
            continue;
        }
        added_points.emplace_back(point_id);
		std::vector<std::string> &point = point_data.at(point_id);

        p.y = std::stof(point[4]);
		p.x = std::stof(point[5]);
		p.z = std::stof(point[3]);

		int_marker_server->insert(makeIntPoint(point_id, p, color));
		int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intMarkerCb, this, _1));
	}
}

void RvizHdmapEditorCore::makeLane()
{
	if (!hdmap_info_dict.at("lane").is_visible) return;
	std::cout << "make lane" << std::endl; 

	geometry_msgs::Point p;
    std::vector<std::string> added_points;
    auto &lane_data = hdmap_info_dict.at("lane").data;
    auto &dtlane_data = hdmap_info_dict.at("dtlane").data;
    auto &point_data = hdmap_info_dict.at("point").data;

	std_msgs::ColorRGBA color;
	color.r = 0.5;
	color.g = 0.0;
	color.b = 0.5;
	color.a = 1.0;

	for (const auto &lane_elem : lane_data)
	{

		std::string point_id = dtlane_data.at(lane_elem.second[1])[2];
        if (std::find(added_points.begin(), added_points.end(), point_id) != added_points.end())
        {
            continue;
        }
        added_points.emplace_back(point_id);
		std::vector<std::string> &point = point_data.at(point_id);

        p.y = std::stof(point[4]);
		p.x = std::stof(point[5]);
		p.z = std::stof(point[3]);

		int_marker_server->insert(makeIntPoint(point_id, p, color));
		int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intMarkerCb, this, _1));
	}
}


visualization_msgs::InteractiveMarker RvizHdmapEditorCore::makeIntPoint(const std::string &name, const geometry_msgs::Point &point, const std_msgs::ColorRGBA &color)
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
	marker.color = color;

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


void RvizHdmapEditorCore::intMarkerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	hdmap_info_dict.at("point").data.at(feedback->marker_name)[3] = std::to_string(feedback->pose.position.z);
	hdmap_info_dict.at("point").data.at(feedback->marker_name)[4] = std::to_string(feedback->pose.position.y);
	hdmap_info_dict.at("point").data.at(feedback->marker_name)[5] = std::to_string(feedback->pose.position.x);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "adas_vis_node");
	RvizHdmapEditorCore adas_vis;
	ros::spin();
	return 0;
}
