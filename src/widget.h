#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QFileDialog>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <pluginlib/class_list_macros.h>
#endif

namespace Ui {
class Widget;
}

namespace rviz_hdmap_builder
{
class Widget : public rviz::Panel
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();

private Q_SLOTS:
    void on_hdmap_ok_clicked();

    void on_hdmap_input_select_clicked();

    void on_hdmap_output_select_clicked();

    void on_waypoint_ok_clicked();

    void on_waypoint_input_select_clicked();

    void on_waypoint_output_select_clicked();

    void on_node_toggled(bool checked);

    void on_lane_toggled(bool checked);

    void on_area_toggled(bool checked);

    void on_roadedge_toggled(bool checked);

    void on_stopline_toggled(bool checked);

    void on_railroad_toggled(bool checked);

    void on_pole_toggled(bool checked);

    void on_crosswalk_toggled(bool checked);

    void on_whiteline_toggled(bool checked);

    void on_intersection_toggled(bool checked);

private:
    Ui::Widget *ui;
    bool hdmap_ok_toggle;
    bool waypoint_ok_toggle;

    QString hdmap_input_filenames;
    QString hdmap_output_filenames;
    QString waypoint_filename;
    QString waypoint_output_filename;

    bool whiteline;
    bool node;
    bool lane;
    bool area;
    bool roadedge;
    bool stopline;
    bool railroad;
    bool pole;
    bool crosswalk;
    bool intersection;

Q_SIGNALS:
    void hdmapInputSelected(QString filenames);
    void hdmapOutputSelected(QString dirname);
    void waypointInputSelected(QString filename);
    void waypointOutputSelected(QString filename);
};
}

#endif // WIDGET_H
