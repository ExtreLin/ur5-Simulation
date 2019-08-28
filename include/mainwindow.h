#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<QThread>
#include<ros/ros.h>
#include<Eigen/Eigen>
#include<moveit/move_group_interface/move_group_interface.h>

namespace Ui {
class MainWindow;
}

class CURControl:public QThread{
    Q_OBJECT
public:
    CURControl();
    ~CURControl();
    void init();
    void addCollision();
    void planningPose(const Eigen::VectorXf &nextPose);
private:
    moveit_msgs::CollisionObject createBox(const Eigen::Vector3f& lwh,const Eigen::Vector3f& boxPose);
    moveit::planning_interface::MoveGroupInterface* group;
    ros::Publisher collision_object_publisher;
    int  boxNum;
};

class CURControlThread : public QThread{
    Q_OBJECT
public:
    CURControlThread(int argc, char** argv):
    argc_(argc),
    argv_(argv),
    kl_(nullptr)
    {

    };

    ~CURControlThread()
    {
         ros::shutdown();
         if(kl_!=nullptr)
            delete kl_;
    }
protected:
    virtual void run();
private:
    int argc_;
    char** argv_;
    CURControl* kl_;
public slots:
    void slotGetPose(const Eigen::VectorXf& nextPose);
    void slotInitGroup();
    void slotAddCollision();
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
      explicit MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    CURControlThread* kLThread_;
public slots:
    void slotGetPose();   
signals:
    void sigSendPose(const Eigen::VectorXf& nextPose);
};

#endif // MAINWINDOW_H
