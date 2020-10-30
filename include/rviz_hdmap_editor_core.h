#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <fstream>
#include <vector>


struct HdmapInfo
{
	std::string input_filename;
	std::string header;
	std::map<std::string ,std::vector<std::string>> data;
	bool is_visible;
};

class RvizHdmapEditorCore
{
private:

	ros::Publisher pub_marker;

    ros::Timer timer;

	// std::map<std::string ,std::vector<std::string>> adas_points;
	// std::map<int, std::vector<std::string>> adas_lines;
	// std::map<int, std::vector<std::string>> adas_whitelines;

	std::map<std::string, HdmapInfo> hdmap_info_dict;

public:
	RvizHdmapEditorCore();
	~RvizHdmapEditorCore();
	void readHdmap(const std::string &filename);
	void saveHdmap(const std::string &dirname);
	void toggleHdmapElement(const std::string &element_name, const bool &toggle);
	void refleshAdasMarker();

private:
	std::string getHdmapType(const std::string &filename);
	visualization_msgs::InteractiveMarker makeIntPoint(const std::string &name, const geometry_msgs::Point &point, const std_msgs::ColorRGBA &color);
	void intMarkerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void makeWhiteline();
	void makeRoadedge();
	void makeStopline();
	void makeNode();
	void makeRailroad();
	void makeCrosswalk();
	void makeIntersection();
	void makePole();
	void makeSignal();
	void makeLane();

};