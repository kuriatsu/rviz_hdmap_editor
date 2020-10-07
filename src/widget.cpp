#include "widget.h"
#include "ui_widget.h"

namespace rviz_hdmap_builder
{
Widget::Widget(QWidget *parent) : 
    rviz::Panel(parent),
    // QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    hdmap_ok_toggle = false;
    waypoint_ok_toggle = false;
}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_hdmap_ok_clicked()
{
    if (!hdmap_ok_toggle)
    {
        ui->hdmap_ok->setText(tr("Save"));

    }
    else
    {
        ui->hdmap_ok->setText(tr("OK"));

    }
    ui->hdmap_input_filename->setEnabled(hdmap_ok_toggle);
    ui->hdmap_input_select->setEnabled(hdmap_ok_toggle);
    ui->hdmap_output_filename->setEnabled(hdmap_ok_toggle);
    ui->hdmap_output_select->setEnabled(hdmap_ok_toggle);
    hdmap_ok_toggle = !hdmap_ok_toggle;
}

void Widget::on_hdmap_input_select_clicked()
{
    QStringList filenames;
    QString filename;
    filenames = QFileDialog::getOpenFileNames(this, tr("Select Files"), "~/", tr("HDMap Files (*.csv)"));

    for (QStringList::Iterator itr = filenames.begin(); itr != filenames.end(); ++itr)
    {
        filename.append(*itr);
        filename.append(",");
    }
    Q_EMIT hdmapInputSelected(filename);
}

void Widget::on_hdmap_output_select_clicked()
{
    QString dirname;
    dirname = QFileDialog::getExistingDirectory(this, tr("Select Dir"), "~/");
    Q_EMIT hdmapOutputSelected(dirname);
}

void Widget::on_waypoint_ok_clicked()
{
    if (!waypoint_ok_toggle)
    {
        ui->waypoint_ok->setText(tr("Save"));

    }
    else
    {
        ui->waypoint_ok->setText(tr("OK"));

    }
    ui->waypoint_input_filename->setEnabled(waypoint_ok_toggle);
    ui->waypoint_input_select->setEnabled(waypoint_ok_toggle);
    ui->waypoint_output_filename->setEnabled(waypoint_ok_toggle);
    ui->waypoint_output_select->setEnabled(waypoint_ok_toggle);
    waypoint_ok_toggle = !waypoint_ok_toggle;
}

void Widget::on_waypoint_input_select_clicked()
{
    QString filename;
    filename = QFileDialog::getOpenFileName(this, tr("Select Files"), "~/", tr("waypoint Files (*.csv)"));
    Q_EMIT waypointInputSelected(filename);
}

void Widget::on_waypoint_output_select_clicked()
{
    QString filename;
    filename = QFileDialog::getOpenFileName(this, tr("Select Files"), "~/", tr("waypoint Files (*.csv)"));
    Q_EMIT waypointOutputSelected(filename);
}

void Widget::on_node_toggled(bool checked)
{
    node = checked;
}

void Widget::on_lane_toggled(bool checked)
{
    lane = checked;
}

void Widget::on_area_toggled(bool checked)
{
    area = checked;
}

void Widget::on_roadedge_toggled(bool checked)
{
    roadedge = checked;
}

void Widget::on_stopline_toggled(bool checked)
{
    stopline = checked;
}

void Widget::on_railroad_toggled(bool checked)
{
    railroad = checked;
}

void Widget::on_pole_toggled(bool checked)
{
    pole = checked;
}

void Widget::on_crosswalk_toggled(bool checked)
{
    crosswalk = checked;
}

void Widget::on_whiteline_toggled(bool checked)
{
    whiteline = checked;
}

void Widget::on_intersection_toggled(bool checked)
{
    intersection = checked;
}
}
PLUGINLIB_EXPORT_CLASS(rviz_hdmap_builder::Widget, rviz::Panel)