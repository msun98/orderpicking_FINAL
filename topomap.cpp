#include "topomap.h"

TOPOMAP::TOPOMAP(QObject *parent) : QObject(parent)
{
    is_loaded = false;
}

TOPOMAP::~TOPOMAP()
{
}

void TOPOMAP::init(UNIMAP* _unimap)
{
    unimap = _unimap;
}

void TOPOMAP::load()
{
    if(setting_config.robot_use_multi)
    {
        QString topo_path = unimap->map_dir + "/topo.json";

        QFile file(topo_path);
        if(file.open(QIODevice::ReadOnly))
        {
            nodes.clear();

            QByteArray data = file.readAll();
            QJsonDocument doc = QJsonDocument::fromJson(data);

            QJsonArray arr = doc.array();
            foreach(const QJsonValue &val, arr)
            {
                NODE *node = new NODE();

                QJsonObject obj = val.toObject();
                node->id = obj["id"].toString().toStdString();
                node->attrib = obj["attrib"].toString().toStdString();
                node->pose = array_to_pose(obj["pose"].toArray());
                node->linked = array_to_linked(obj["linked"].toArray());

                nodes[node->id] = node;
            }
            file.close();

            is_loaded = true;

            printf("[TOPOMAP] %s loaded\n", topo_path.toStdString().c_str());
        }
    }
}

void TOPOMAP::draw_topo_nodes(cv::Mat &img)
{
    int node_r = static_config.robot_radius/unimap->map_grid_width;

    for(auto& it: nodes)
    {
        if(it.second == nullptr)
        {
            continue;
        }

        cv::Vec3d pose = it.second->pose;
        cv::Vec2i uv0 = unimap->xy_uv(cv::Vec2d(0, 0), pose);
        cv::Vec2i uv1 = unimap->xy_uv(cv::Vec2d(static_config.robot_radius, 0), pose);

        // draw node
        if(it.second->attrib == "ROUTE")
        {
            cv::circle(img, cv::Point(uv0[0], uv0[1]), node_r, cv::Scalar(196,196,196), 1, cv::LineTypes::LINE_AA);
        }
        else if(it.second->attrib == "SERVING")
        {
            cv::circle(img, cv::Point(uv0[0], uv0[1]), node_r, cv::Scalar(0,255,0), 1, cv::LineTypes::LINE_AA);
            cv::line(img, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), cv::Scalar(0,255,0), 1, cv::LineTypes::LINE_AA);
        }
        else if(it.second->attrib == "CHARGING")
        {
            cv::circle(img, cv::Point(uv0[0], uv0[1]), node_r, cv::Scalar(0,0,128), 1, cv::LineTypes::LINE_AA);
            cv::line(img, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), cv::Scalar(0,0,128), 1, cv::LineTypes::LINE_AA);
        }
        else if(it.second->attrib == "RESTING")
        {
            cv::circle(img, cv::Point(uv0[0], uv0[1]), node_r, cv::Scalar(255,255,255), 1, cv::LineTypes::LINE_AA);
            cv::line(img, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), cv::Scalar(255,255,255), 1, cv::LineTypes::LINE_AA);
        }
        else if(it.second->attrib == "TEMP")
        {
            cv::circle(img, cv::Point(uv0[0], uv0[1]), node_r, cv::Scalar(255, 191, 0), 1, cv::LineTypes::LINE_AA);
            cv::line(img, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), cv::Scalar(255, 191, 0), 1, cv::LineTypes::LINE_AA);
        }

        // draw edge
        for(size_t p = 0; p < it.second->linked.size(); p++)
        {
            std::string id = it.second->linked[p];
            cv::Vec2i uv1 = unimap->xy_uv(get_pos(id));

            double dx = uv1[0] - uv0[0];
            double dy = uv1[1] - uv0[1];
            double d = std::sqrt(dx*dx + dy*dy);
            double nx = dx/d;
            double ny = dy/d;

            cv::Point pt0;
            pt0.x = uv0[0] + nx*node_r;
            pt0.y = uv0[1] + ny*node_r;

            cv::Point pt1;
            pt1.x = uv0[0] + nx*(d-node_r);
            pt1.y = uv0[1] + ny*(d-node_r);

            double tip = (double)node_r/d;
            cv::arrowedLine(img, pt0, pt1, cv::Scalar(196,196,196), 1, cv::LineTypes::LINE_AA, 0, tip);
        }
    }
}

std::string TOPOMAP::get_node_nn(cv::Vec2d pt)
{
    std::string min_id = "";
    double min_d = 99999999;
    for(auto& it: nodes)
    {
        if(it.second == nullptr)
        {
            continue;
        }

        double d = cv::norm(get_pos(it.second->id) - pt);
        if(d < min_d)
        {
            min_d = d;
            min_id = it.second->id;
        }
    }
    return min_id;
}

std::string TOPOMAP::get_node_loosly(cv::Vec2d pt)
{
    std::string min_id = "";
    double min_d = 99999999;
    for(auto& it: nodes)
    {
        if(it.second == nullptr)
        {
            continue;
        }

        double d = cv::norm(get_pos(it.second->id) - pt);
        if(d < min_d && d <= static_config.robot_radius*2.0)
        {
            min_d = d;
            min_id = it.second->id;
        }
    }
    return min_id;
}

std::string TOPOMAP::get_node_tightly(cv::Vec2d pt)
{
    std::string min_id = "";
    double min_d = 99999999;
    for(auto& it: nodes)
    {
        if(it.second == nullptr)
        {
            continue;
        }

        double d = cv::norm(get_pos(it.second->id) - pt);
        if(d < min_d && d <= static_config.robot_radius)
        {
            min_d = d;
            min_id = it.second->id;
        }
    }
    return min_id;
}

std::string TOPOMAP::get_node_with_edge(cv::Vec2d pt)
{
    std::string res;
    for(auto& it: nodes)
    {
        if(it.second == nullptr)
        {
            continue;
        }

        cv::Vec2d pos0 = get_pos(it.second->id);

        bool is_found = false;
        for(size_t p = 0; p < it.second->linked.size(); p++)
        {
            cv::Vec2d pos1 = get_pos(it.second->linked[p]);

            double x_min = std::min<double>(pos0[0], pos1[0])-static_config.robot_radius;
            double x_max = std::max<double>(pos0[0], pos1[0])+static_config.robot_radius;
            double y_min = std::min<double>(pos0[1], pos1[1])-static_config.robot_radius;
            double y_max = std::max<double>(pos0[1], pos1[1])+static_config.robot_radius;
            if(pt[0] > x_min && pt[0] < x_max && pt[1] > y_min && pt[1] < y_max)
            {
                double d0 = cv::norm(pos0 - pt);
                double d1 = cv::norm(pos1 - pt);
                if(d0 < d1)
                {
                    res = it.second->id;
                    is_found = true;
                    break;
                }
                else
                {
                    res = nodes[it.second->linked[p]]->id;
                    is_found = true;
                    break;
                }
            }
        }

        if(is_found)
        {
            break;
        }
    }
    return res;
}

cv::Vec2d TOPOMAP::get_pos(std::string id)
{
    return cv::Vec2d(nodes[id]->pose[0], nodes[id]->pose[1]);
}

cv::Vec3d TOPOMAP::get_pose(std::string id)
{
    return nodes[id]->pose;
}
