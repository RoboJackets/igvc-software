#ifndef PATHADAPTER_H
#define PATHADAPTER_H
#include <intelligence/pathplanning/astarplanner.h>
#include <QWidget>

namespace Ui {
class PathAdapter;
}

class PathAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit PathAdapter(QWidget *parent = 0);
    ~PathAdapter();

protected:
    void wheelEvent(QWheelEvent* event);
    void mouseMoveEvent(QMouseEvent *);

private:
    Ui::PathAdapter *ui;
    path_t path;
    bool pathRecieved;
    double scale;
    QPoint center;

    void paintEvent(QPaintEvent *);
    void drawArc(QPainter *painter, SearchLocation dest, SearchMove moveTaken, double scale, QPoint origin);

signals:
    void setStart(RobotPosition pos);
    void setEnd(RobotPosition pos);
    void setMap(pcl::PointCloud<pcl::PointXYZ>::Ptr map);

public slots:
    void newPath(path_t)
    {
        std::cout << "Path of length " << path.size() << " recieved." << std::endl;
        pathRecieved = true;
    }
};

#endif // PATHADAPTER_H
