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
    if (waypoint_ok_toggle)
    {
        editor_core.saveWaypoint(ui->waypoint_output_filename->text().toStdString());
        ui->waypoint_ok->setText(tr("OK"));
        editor_core.clearWaypoint();
        ui->waypoint_input_filename->setEnabled(waypoint_ok_toggle);
        ui->waypoint_input_select->setEnabled(waypoint_ok_toggle);
        ui->waypoint_output_filename->setEnabled(waypoint_ok_toggle);
        ui->waypoint_output_select->setEnabled(waypoint_ok_toggle);
        waypoint_ok_toggle = !waypoint_ok_toggle;
    }

    if (!hdmap_ok_toggle)
    {
        if (ui->hdmap_input_filename->text().isEmpty() || ui->hdmap_output_filename->text().isEmpty()) return;

        for (auto &e: ui->hdmap_input_filename->text().split(QLatin1Char(',')))
        {
            std::string type = editor_core.readHdmap(e.toStdString());
            // if (std::stoi(type) == 0) continue;
            
            if      (type == "node")         ui->node->setEnabled(true);
            else if (type == "lane")         ui->lane->setEnabled(true);
            else if (type == "area")         ui->area->setEnabled(true);
            else if (type == "roadedge")     ui->roadedge->setEnabled(true);
            else if (type == "stopline")     ui->stopline->setEnabled(true);
            else if (type == "railroad_crossing")     ui->railroad->setEnabled(true);
            else if (type == "pole")         ui->pole->setEnabled(true);
            else if (type == "signaldata")       ui->signal->setEnabled(true);
            else if (type == "crosswalk")    ui->crosswalk->setEnabled(true);
            else if (type == "whiteline")    ui->whiteline->setEnabled(true);
            else if (type == "intersection") ui->intersection->setEnabled(true);
        }
        ui->hdmap_ok->setText(tr("Save"));
        // editor_core.refleshAdasMarker();
    }
    else
    {
        editor_core.saveHdmap(ui->hdmap_output_filename->text().toStdString());
        ui->hdmap_ok->setText(tr("OK"));

        ui->node->setEnabled(false);
        ui->lane->setEnabled(false);
        ui->area->setEnabled(false);
        ui->roadedge->setEnabled(false);
        ui->stopline->setEnabled(false);
        ui->railroad->setEnabled(false);
        ui->pole->setEnabled(false);
        ui->signal->setEnabled(false);
        ui->crosswalk->setEnabled(false);
        ui->whiteline->setEnabled(false);
        ui->intersection->setEnabled(false);

        editor_core.clearAdas();

    }
    ui->hdmap_input_filename->setEnabled(hdmap_ok_toggle);
    ui->hdmap_input_select->setEnabled(hdmap_ok_toggle);
    ui->hdmap_output_filename->setEnabled(hdmap_ok_toggle);
    ui->hdmap_output_select->setEnabled(hdmap_ok_toggle);
    hdmap_ok_toggle = !hdmap_ok_toggle;
}

void Widget::on_hdmap_input_select_clicked()
{
    QStringList filename_list;
    QString filename;
    filename_list = QFileDialog::getOpenFileNames(this, tr("Select Files"), "~/", tr("HDMap Files (*.csv)"));

    for (QStringList::Iterator itr = filename_list.begin(); itr != filename_list.end(); ++itr)
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
    if(hdmap_ok_toggle)
    {
        editor_core.saveHdmap(ui->hdmap_output_filename->text().toStdString());
        ui->hdmap_ok->setText(tr("OK"));

        ui->node->setEnabled(false);
        ui->lane->setEnabled(false);
        ui->area->setEnabled(false);
        ui->roadedge->setEnabled(false);
        ui->stopline->setEnabled(false);
        ui->railroad->setEnabled(false);
        ui->pole->setEnabled(false);
        ui->signal->setEnabled(false);
        ui->crosswalk->setEnabled(false);
        ui->whiteline->setEnabled(false);
        ui->intersection->setEnabled(false);

        editor_core.clearAdas();

        ui->hdmap_input_filename->setEnabled(hdmap_ok_toggle);
        ui->hdmap_input_select->setEnabled(hdmap_ok_toggle);
        ui->hdmap_output_filename->setEnabled(hdmap_ok_toggle);
        ui->hdmap_output_select->setEnabled(hdmap_ok_toggle);
        hdmap_ok_toggle = !hdmap_ok_toggle;
    }

    if (!waypoint_ok_toggle)
    {
        if (ui->waypoint_input_filename->text().isEmpty() || ui->waypoint_output_filename->text().isEmpty()) return;
        editor_core.readWaypoint(ui->waypoint_input_filename->text().toStdString());
        ui->waypoint_ok->setText(tr("Save"));

    }
    else
    {
        editor_core.saveWaypoint(ui->waypoint_output_filename->text().toStdString());
        ui->waypoint_ok->setText(tr("OK"));
        editor_core.clearWaypoint();

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
    editor_core.toggleHdmapElement("node", checked);
}

void Widget::on_lane_toggled(bool checked)
{
    editor_core.toggleHdmapElement("lane", checked);
}

void Widget::on_area_toggled(bool checked)
{
    editor_core.toggleHdmapElement("area", checked);
}

void Widget::on_roadedge_toggled(bool checked)
{
    editor_core.toggleHdmapElement("roadedge", checked);
}

void Widget::on_stopline_toggled(bool checked)
{
    editor_core.toggleHdmapElement("stopline", checked);
}

void Widget::on_railroad_toggled(bool checked)
{
    editor_core.toggleHdmapElement("railroad_crossing", checked);
}

void Widget::on_pole_toggled(bool checked)
{
    editor_core.toggleHdmapElement("pole", checked);
}

void Widget::on_crosswalk_toggled(bool checked)
{
    editor_core.toggleHdmapElement("crosswalk", checked);
}

void Widget::on_whiteline_toggled(bool checked)
{
    editor_core.toggleHdmapElement("whiteline", checked);
}

void Widget::on_intersection_toggled(bool checked)
{
    editor_core.toggleHdmapElement("intersection", checked);
}

void Widget::on_signal_toggled(bool checked)
{
    editor_core.toggleHdmapElement("signaldata", checked);
}
}
PLUGINLIB_EXPORT_CLASS(rviz_hdmap_builder::Widget, rviz::Panel)