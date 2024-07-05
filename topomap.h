#ifndef TOPOMAP_H
#define TOPOMAP_H

#include "global_defines.h"

#include "unimap.h"

#include <QObject>
#include <QSettings>
#include <QFileInfo>
#include <QByteArray>
#include <QJsonDocument>
#include <QJsonValue>
#include <QJsonArray>
#include <QJsonObject>

class TOPOMAP : public QObject
{
    Q_OBJECT
public:
    // topomap node
    struct NODE
    {
        std::string id;
        std::string attrib; // ROUTE or GOAL
        cv::Vec3d pose;
        std::vector<std::string> linked;

        NODE()
        {
            id = "";
            attrib = "ROUTE";
            pose = cv::Vec3d(0,0,0);
            linked.clear();
        }

        NODE(const NODE& p)
        {
            id = p.id;
            attrib = p.attrib;
            pose = p.pose;
            linked = p.linked;
        }

        NODE& operator=(const NODE& p)
        {
            id = p.id;
            attrib = p.attrib;
            pose = p.pose;
            linked = p.linked;
            return *this;
        }
    };

    // json interface
    QJsonArray pose_to_array(cv::Vec3d pose)
    {
        QJsonArray res;
        res.append(pose[0]);
        res.append(pose[1]);
        res.append(pose[2]);
        return res;
    }

    cv::Vec3d array_to_pose(QJsonArray arr)
    {
        cv::Vec3d res;
        res[0] = arr[0].toDouble();
        res[1] = arr[1].toDouble();
        res[2] = arr[2].toDouble();
        return res;
    }

    QJsonArray linked_to_array(std::vector<std::string> linked)
    {
        QJsonArray res;
        for(size_t p = 0; p < linked.size(); p++)
        {
            res.append(linked[p].c_str());
        }
        return res;
    }

    std::vector<std::string> array_to_linked(QJsonArray arr)
    {
        std::vector<std::string> res;
        for(int p = 0; p < arr.size(); p++)
        {
            res.push_back(arr[p].toString().toStdString());
        }
        return res;
    }

public:
    explicit TOPOMAP(QObject *parent = nullptr);
    ~TOPOMAP();

    void init(UNIMAP* _unimap);
    void load();

    std::string get_node_nn(cv::Vec2d pt);
    std::string get_node_loosly(cv::Vec2d pt);
    std::string get_node_tightly(cv::Vec2d pt);
    std::string get_node_with_edge(cv::Vec2d pt);

    cv::Vec2d get_pos(std::string id);
    cv::Vec3d get_pose(std::string id);
    void draw_topo_nodes(cv::Mat &img);

    std::atomic<bool> is_loaded;

    // storage
    std::map<std::string, NODE*> nodes;

private:
    UNIMAP *unimap;

signals:

};

#endif // TOPOMAP_H
