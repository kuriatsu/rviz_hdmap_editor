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


std::string RvizHdmapEditorCore::readHdmap(const std::string &filename)
{
	// if (filename.empty()) return 0;

	std::ifstream ifs(filename);
	std::string line, field, header, type = getHdmapType(filename);
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

	hdmap_info.data = extracted_data;
	hdmap_info_dict[type] = hdmap_info;
	return type;

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

			if(hdmap_info_dict.at("whiteline").is_visible)
			{
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
				int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intPointMarkerCb, this, _1));		
			}
			else
			{
				int_marker_server->erase(point_id);
			}
		}
	}
}

void RvizHdmapEditorCore::makeRoadedge()
{
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

			if (hdmap_info_dict.at("roadedge").is_visible)
			{
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
				int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intPointMarkerCb, this, _1));		
			}
         	else
         	{
         		int_marker_server->erase(point_id);
         	}
		}
	}
}

void RvizHdmapEditorCore::makeStopline()
{
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

			if (hdmap_info_dict.at("stopline").is_visible)
			{
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
				int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intPointMarkerCb, this, _1));		
			}
			else
			{
				int_marker_server->erase(point_id);
			}
		}
	}
}


void RvizHdmapEditorCore::makeNode()
{
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

		if (hdmap_info_dict.at("node").is_visible)
		{
		    if (std::find(added_points.begin(), added_points.end(), point_id) != added_points.end())
	        {
	            continue;
	        }

			std::vector<std::string> &point = point_data.at(point_id);
			added_points.emplace_back(point_id);

	        p.y = std::stof(point[4]);
			p.x = std::stof(point[5]);
			p.z = std::stof(point[3]);
			// std::cout << node_elem.second[0] << std::endl;

			int_marker_server->insert(makeIntPoint(point_id, p, color));
			int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intPointMarkerCb, this, _1));
		
		}
		else
		{
			int_marker_server->erase(point_id);
		}    

	}
}

void RvizHdmapEditorCore::makeRailroad()
{
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
		 
				if(hdmap_info_dict.at("railroad_crossing").is_visible)
				{
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
					int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intPointMarkerCb, this, _1));	
				}
				else
				{
					int_marker_server->erase(point_id);
				}
		        
			}
		}
	}
}


void RvizHdmapEditorCore::makeCrosswalk()
{
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

				if(hdmap_info_dict.at("crosswalk").is_visible)
				{
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
					int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intPointMarkerCb, this, _1));	
				}
		        else
		        {
		        	int_marker_server->erase(point_id);
		        }
			}
		}
	}
}


void RvizHdmapEditorCore::makeIntersection()
{
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
		 
				if(hdmap_info_dict.at("intersection").is_visible)
				{
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
					int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intPointMarkerCb, this, _1));	
				}
				else
				{
					int_marker_server->erase(point_id);
				}
		        
			}
		}
	}
}


void RvizHdmapEditorCore::makePole()
{
	std::cout << "make pole" << std::endl; 

	geometry_msgs::Pose p;
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
        
		if (hdmap_info_dict.at("pole").is_visible)
		{
			if (std::find(added_points.begin(), added_points.end(), point_id) != added_points.end())
	        {
	            continue;
	        }

	        added_points.emplace_back(point_id);
			std::vector<std::string> &point = point_data.at(point_id);

	        p.position.y = std::stof(point[4]);
			p.position.x = std::stof(point[5]);
			p.position.z = std::stof(point[3]);
			p.orientation = rpy2Quat(std::stod(vector_data.at(pole_elem.second[1])[3]), 0.0, std::stod(vector_data.at(pole_elem.second[1])[2]));

			int_marker_server->insert(makeIntVector(point_id, p, color));
			int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intPointMarkerCb, this, _1));	
		}
		else
		{
			int_marker_server->erase(point_id);
		}
	}
}

void RvizHdmapEditorCore::makeSignal()
{
	std::cout << "make signaldata" << std::endl; 

	geometry_msgs::Pose p;
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

		if (hdmap_info_dict.at("signaldata").is_visible)
		{
			if (std::find(added_points.begin(), added_points.end(), point_id) != added_points.end())
	        {
	            continue;
	        }
	        added_points.emplace_back(point_id);
			std::vector<std::string> &point = point_data.at(point_id);

	        p.position.y = std::stof(point[4]);
			p.position.x = std::stof(point[5]);
			p.position.z = std::stof(point[3]);
			p.orientation = rpy2Quat(std::stod(vector_data.at(signal_elem.second[1])[3]), 0.0, std::stod(vector_data.at(signal_elem.second[1])[2]));
			int_marker_server->insert(makeIntVector(point_id, p, color));
			int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intPointMarkerCb, this, _1));	
		}
		else
		{
			int_marker_server->erase(point_id);
		}
	}
}

void RvizHdmapEditorCore::makeLane()
{
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

		if (hdmap_info_dict.at("lane").is_visible)
		{
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
			int_marker_server->setCallback(point_id, boost::bind(&RvizHdmapEditorCore::intPointMarkerCb, this, _1));
			
		}
		else
		{
			int_marker_server->erase(point_id);
		}		
	}
}


void RvizHdmapEditorCore::readWaypoint(const std::string &filename)
{
	// if (filename.empty()) return 0;

	std::ifstream ifs(filename);
	std::string line, field, header;
	std::vector<std::string> raw_data_list;
	std::map<std::string, std::vector<std::string>> extracted_data;
	HdmapInfo hdmap_info;
	int count = 0;

	hdmap_info.input_filename = filename;
	hdmap_info.is_visible = true;
	std::getline(ifs, hdmap_info.header);
	ROS_INFO_STREAM(filename);

	while(std::getline(ifs, line))
	{
		std::istringstream stream(line);
		while(std::getline(stream, field, ','))
		{
			raw_data_list.emplace_back(field);
		}
		extracted_data[std::to_string(count)] = raw_data_list;
		raw_data_list.clear();
		count++;
	}	

	hdmap_info.data = extracted_data;
	hdmap_info_dict["waypoint"] = hdmap_info;

	makeWaypoint();
	int_marker_server->applyChanges();

}


void RvizHdmapEditorCore::saveWaypoint(const std::string &filename)
{
	std::cout << filename << std::endl;
	std::ofstream ofs(filename);

	ofs << hdmap_info_dict.at("waypoint").header << std::endl;

	for(auto &data_elem: hdmap_info_dict.at("waypoint").data)
	{
		for (int j = 0; j < data_elem.second.size() - 1 ; j++)
		{
			ofs << data_elem.second[j] << ",";
		}
		ofs << data_elem.second.back() << std::endl;
	}
}


void RvizHdmapEditorCore::makeWaypoint()
{
	std::cout << "make waypoint" << std::endl; 

	geometry_msgs::Pose p;
    std::vector<std::string> added_points;

	std_msgs::ColorRGBA color;
	color.r = 0.5;
	color.g = 0.2;
	color.b = 0.2;
	color.a = 1.0;

	for (const auto &wp_elem : hdmap_info_dict.at("waypoint").data)
	{
		if (hdmap_info_dict.at("waypoint").is_visible)
		{
	        if (std::find(added_points.begin(), added_points.end(), wp_elem.first) != added_points.end())
	        {
	            continue;
	        }

	        added_points.emplace_back(wp_elem.first);

			p.position.x = std::stof(wp_elem.second[0]);
	        p.position.y = std::stof(wp_elem.second[1]);
			p.position.z = std::stof(wp_elem.second[2]);
			p.orientation = rpy2Quat(0.0 ,0.0, std::stof(wp_elem.second[3]));
			int_marker_server->insert(makeIntVector(wp_elem.first, p, color));
			int_marker_server->setCallback(wp_elem.first, boost::bind(&RvizHdmapEditorCore::intWaypointMarkerCb, this, _1));
		}
		else
		{
			int_marker_server->erase(wp_elem.first);
		}
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
	control.name = "move plane";
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


visualization_msgs::InteractiveMarker RvizHdmapEditorCore::makeIntVector(const std::string &name, const geometry_msgs::Pose &pose, const std_msgs::ColorRGBA &color)
{

	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = "map";
	int_marker.name = name;
	int_marker.scale = 0.1;
	int_marker.pose = pose;

	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible = true;
	visualization_msgs::Marker marker;
	marker.ns="point";
	marker.id = std::stoi(name);
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	marker.color = color;
	control.markers.emplace_back(marker);
    control.name = "move_3d";
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

	visualization_msgs::InteractiveMarkerControl move_control;
   	move_control.orientation.w = 1;
    move_control.orientation.x = 0;
    move_control.orientation.y = 1;
    move_control.orientation.z = 0;

    move_control.name = "rotate_z";
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(move_control);

   	// move_control.orientation.w = 1;
    // move_control.orientation.x = 0;
    // move_control.orientation.y = 1;
    // move_control.orientation.z = 0;
    // move_control.name = "move_z";
    // move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    // int_marker.controls.push_back(move_control);

	return int_marker;
}


geometry_msgs::Quaternion RvizHdmapEditorCore::rpy2Quat(const double roll, const double pitch, const double yaw)
{
	tf::Quaternion tf_quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
	geometry_msgs::Quaternion gm_quat;
	quaternionTFToMsg(tf_quat, gm_quat);
	return gm_quat;
}


double RvizHdmapEditorCore::quat2Yaw(geometry_msgs::Quaternion gm_quat)
{
	double roll, pitch, yaw;
	tf::Quaternion tf_quat;
	quaternionMsgToTF(gm_quat, tf_quat);
	tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
	return yaw;
}


void RvizHdmapEditorCore::intPointMarkerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	hdmap_info_dict.at("point").data.at(feedback->marker_name)[3] = std::to_string(feedback->pose.position.z);
	hdmap_info_dict.at("point").data.at(feedback->marker_name)[4] = std::to_string(feedback->pose.position.y);
	hdmap_info_dict.at("point").data.at(feedback->marker_name)[5] = std::to_string(feedback->pose.position.x);
}


void RvizHdmapEditorCore::intVectorMarkerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	hdmap_info_dict.at("point").data.at(feedback->marker_name)[3] = std::to_string(feedback->pose.position.z);
	hdmap_info_dict.at("point").data.at(feedback->marker_name)[4] = std::to_string(feedback->pose.position.y);
	hdmap_info_dict.at("point").data.at(feedback->marker_name)[5] = std::to_string(feedback->pose.position.x);
	for (auto &vector_elem : hdmap_info_dict.at("vector").data)
	{
		// only signal data has some value at Hang (vang=0), poll should be hang=0, vang=0
		if (vector_elem.second[1] == feedback->marker_name && vector_elem.second[3] != std::to_string(0))
		{
			vector_elem.second[2] = std::to_string(quat2Yaw(feedback->pose.orientation));
			break;
		}
	}
}


void RvizHdmapEditorCore::intWaypointMarkerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	hdmap_info_dict.at("waypoint").data.at(feedback->marker_name)[2] = std::to_string(feedback->pose.position.z);
	hdmap_info_dict.at("waypoint").data.at(feedback->marker_name)[1] = std::to_string(feedback->pose.position.y);
	hdmap_info_dict.at("waypoint").data.at(feedback->marker_name)[0] = std::to_string(feedback->pose.position.x);
	hdmap_info_dict.at("waypoint").data.at(feedback->marker_name)[3] = std::to_string(quat2Yaw(feedback->pose.orientation));
}


void RvizHdmapEditorCore::clearAdas()
{

	for (auto &hdmap_info : hdmap_info_dict)
	{
		if (hdmap_info.first == "waypoint") continue;
		hdmap_info.second.is_visible = false;
		refleshAdasMarker();
		hdmap_info_dict.erase(hdmap_info.first);
	}

}

void RvizHdmapEditorCore::clearWaypoint()
{
	if (!hdmap_info_dict.count("waypoint")) return;
	hdmap_info_dict.at("waypoint").is_visible = false;
	makeWaypoint();
	int_marker_server->applyChanges();
	hdmap_info_dict.erase("waypoint");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "adas_vis_node");
	RvizHdmapEditorCore adas_vis;
	ros::spin();
	return 0;
}
