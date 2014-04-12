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

private:
    Ui::PathAdapter *ui;
    path_t path;
    void paintEvent(QPaintEvent *);
    bool pathRecieved;
    double scale;

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
