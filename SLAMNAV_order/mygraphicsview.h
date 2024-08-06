#pragma once

// Global definition
#include "global_defines.h"

// QT
#include <QGraphicsView>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QGraphicsItem>

// STL
#include <atomic>
#include <math.h>

class MyGraphicsView : public QGraphicsView
{
	Q_OBJECT

public:
	MyGraphicsView(QWidget *parent);
	~MyGraphicsView();

    void reload_gv_screen();
    void update_map_info(int _map_size);

	QGraphicsPixmapItem map;
    QGraphicsRectItem *rect;

    cv::Vec3d pre_target_pose;
    cv::Vec3d target_pose;

    int map_size = 1000;
    double grid_size = 0.05;

private:
    QGraphicsScene scene;
    QGraphicsLineItem line;

    bool isDragL = false;
    bool isDragR = false;
    bool isGrab = false;

    double x0 = 0;
    double y0 = 0;

    QPointF pt0;

signals:
    void pose_clicked(double x, double y, double th); // x(m), y(m), th(rad)

protected:
	void wheelEvent(QWheelEvent* ev);
	void mousePressEvent(QMouseEvent *ev);
	void mouseMoveEvent(QMouseEvent *ev);
	void mouseReleaseEvent(QMouseEvent *ev);
	void keyPressEvent(QKeyEvent *ev);
	void keyReleaseEvent(QKeyEvent *ev);
};
