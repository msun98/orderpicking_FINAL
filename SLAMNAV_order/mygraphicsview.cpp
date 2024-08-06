#include "mygraphicsview.h"

MyGraphicsView::MyGraphicsView(QWidget *parent)
    : QGraphicsView(parent)
    , scene(this)
{
	setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);	
	setScene(&scene);
	
    scene.setBackgroundBrush(Qt::black);
    scene.addItem(&map);

    rect = new QGraphicsRectItem(0, 0, map_size, map_size);
    rect->setPen(QPen(QBrush(Qt::yellow), 3));
    scene.addItem(rect);

    line.setPen(QPen(Qt::yellow, 1, Qt::DotLine));
    line.setLine(QLineF(0,0,0,0));
    scene.addItem(&line);

    pre_target_pose = cv::Vec3d(0,0,0);
    target_pose = cv::Vec3d(0,0,0);
}

MyGraphicsView::~MyGraphicsView()
{
    scene.removeItem(&map);
}

void MyGraphicsView::reload_gv_screen()
{
    scene.removeItem(rect);
    rect = new QGraphicsRectItem(0, 0, map_size, map_size);
    rect->setPen(QPen(QBrush(Qt::yellow), 3));
    scene.addItem(rect);
}

void MyGraphicsView::update_map_info(int _map_size)
{
    map_size = _map_size;
}

void MyGraphicsView::wheelEvent(QWheelEvent * ev)
{
	// Scale the view / do the zoom
	double scaleFactor = 1.05;
	if (ev->delta() > 0)
	{
		// Zoom in
		scale(scaleFactor, scaleFactor);
	}
	else
	{
		// Zooming out
		scale(1.0 / scaleFactor, 1.0 / scaleFactor);
	}

	//QGraphicsView::wheelEvent(ev);
}

void MyGraphicsView::mousePressEvent(QMouseEvent * ev)
{
    if (ev->button() == Qt::LeftButton && !isGrab)
	{
        QPointF pt = mapToScene(ev->pos());

        // for guide line
        pt0 = pt;        
        line.setLine(QLineF(pt, pt));
        line.show();

        // for desired robot pose
        x0 = -(pt.y()-(map_size/2))*grid_size;
        y0 = -(pt.x()-(map_size/2))*grid_size;
		isDragL = true;
	}
	
    if (ev->button() == Qt::RightButton && !isGrab)
	{
        QPointF pt = mapToScene(ev->pos());
		isDragR = true;
	}

	QGraphicsView::mousePressEvent(ev);
}

void MyGraphicsView::mouseMoveEvent(QMouseEvent * ev)
{
	if (isDragL)
	{
        QPointF pt = mapToScene(ev->pos());        
        line.setLine(QLineF(pt0, pt));
	}
	
	if (isDragR)
	{
        QPointF pt = mapToScene(ev->pos());
	}

	QGraphicsView::mouseMoveEvent(ev);
}

void MyGraphicsView::mouseReleaseEvent(QMouseEvent * ev)
{
	if (ev->button() == Qt::LeftButton)
	{
		isDragL = false;
        line.hide();

        QPointF pt = mapToScene(ev->pos());
        double x1 = -(pt.y()-(map_size/2))*grid_size;
        double y1 = -(pt.x()-(map_size/2))*grid_size;
        double x = x0;
        double y = y0;
        double th = std::atan2(y1-y0, x1-x0);

        if(!isGrab)
        {
            pre_target_pose = target_pose;
            target_pose = cv::Vec3d(x, y, th);
            emit pose_clicked(x, y, th);
            //printf("x:%f, y:%f, th:%f\n", x, y, th*R2D);
        }
	}

	if (ev->button() == Qt::RightButton)
	{
		isDragR = false;
        QPointF pt = mapToScene(ev->pos());
	}

	QGraphicsView::mouseReleaseEvent(ev);
}

void MyGraphicsView::keyPressEvent(QKeyEvent * ev)
{
	if (ev->isAutoRepeat())
	{
		return;
	}

    if (ev->key() == Qt::Key_Shift)
    {
        isGrab = true;
		setDragMode(ScrollHandDrag);
		return;
	}
}

void MyGraphicsView::keyReleaseEvent(QKeyEvent * ev)
{
	if (ev->isAutoRepeat())
	{
		return;
	}

    if (ev->key() == Qt::Key_Shift)
	{
		setDragMode(NoDrag);
        isGrab = false;
		return;
	}
}
