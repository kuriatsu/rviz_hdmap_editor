#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>


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
    // ros::Timer timer;
	std::map<std::string, HdmapInfo> hdmap_info_dict;

public:
	RvizHdmapEditorCore();
	~RvizHdmapEditorCore();
	std::string readHdmap(const std::string &filename);
	void readWaypoint(const std::string &filename);
	void saveHdmap(const std::string &dirname);
	void saveWaypoint(const std::string &filename);
	void toggleHdmapElement(const std::string &element_name, const bool &toggle);
	void clearAdas();
	void clearWaypoint();

private:
	std::string getHdmapType(const std::string &filename);
	visualization_msgs::InteractiveMarker makeIntPoint(const std::string &name, const geometry_msgs::Point &point, const std_msgs::ColorRGBA &color);
	visualization_msgs::InteractiveMarker makeIntVector(const std::string &name, const geometry_msgs::Pose &pose, const std_msgs::ColorRGBA &color);
	void refleshAdasMarker();
	void intPointMarkerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void intVectorMarkerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void intWaypointMarkerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	geometry_msgs::Quaternion rpy2Quat(const double roll, const double pitch, const double yaw);
	double quat2Yaw(geometry_msgs::Quaternion gm_quat);
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
	void makeWaypoint();

};