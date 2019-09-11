#include "mainwindow.h"
#include "ui_mainwindow.h"
#include<moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <eigen_conversions/eigen_msg.h>
#include<boost/shared_ptr.hpp>
#include<QMessageBox>

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->lineEditpx->setText("0.5");
    ui->lineEditpy->setText("0");
    ui->lineEditpz->setText("0");
    ui->lineEditrx->setText("0");
    ui->lineEditry->setText("0");
    ui->lineEditrz->setText("0");
    ui->lineEditrw->setText("1");

    kLThread_ = new CURControlThread(argc,argv);
    connect(this,SIGNAL(sigSendPose(const Eigen::VectorXf& )),kLThread_,SLOT(slotGetPose(const Eigen::VectorXf& )));
    connect(ui->planBtn ,SIGNAL(clicked()),this,SLOT(slotGetPose()));
    connect(ui->initBtn,SIGNAL(clicked()),kLThread_,SLOT(slotInitGroup()));
    connect(ui->addCollisionBtn,SIGNAL(clicked()),kLThread_,SLOT(slotAddCollision()));
    kLThread_->start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow:: slotGetPose()
{ 
    Eigen::VectorXf pos(7);
    pos << ui->lineEditpx->text().toFloat(),  
                  ui->lineEditpy->text().toFloat(),
                  ui->lineEditpz->text().toFloat(),
                  ui->lineEditrx->text().toFloat(),
                  ui->lineEditry->text().toFloat(),
                  ui->lineEditrz->text().toFloat(),
                  ui->lineEditrw->text().toFloat();
    emit sigSendPose(pos);
}


CURControl::CURControl():group(nullptr),boxNum(0)
{}

CURControl::~CURControl()
{
    if(group!=nullptr)
        delete group;
}

void CURControl::init()
{
    group = new   moveit::planning_interface::MoveGroupInterface ("manipulator");
    ros::NodeHandle node_handle;
    planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
   	//ros::Publisher collision_state_pub = node_handle.advertise<collision_detection::Collision>("collision_detection", 1);
    QMessageBox::information(nullptr,"info","init finished !",QMessageBox::Ok);
}

 moveit_msgs::CollisionObject CURControl::createBox(const Eigen::Vector3f& lwh,const Eigen::Vector3f& boxPose)
{
       moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = group->getPlanningFrame();
        collision_object.id = "box"+std::to_string(boxNum);
        boxNum++;
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        
        primitive.dimensions[0] = lwh[0];
        primitive.dimensions[1] = lwh[1];
        primitive.dimensions[2] = lwh[2];

        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x =  boxPose[0];
        box_pose.position.y = boxPose[1];
        box_pose.position.z =  boxPose[2];

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;
        return collision_object;
}

void CURControl::addCollision()
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    sleep(3.0);
    //添加box
    moveit_msgs::AttachedCollisionObject collision_object4;
    collision_object4.link_name = group->getEndEffectorLink();
    collision_object4.object.header.frame_id =  group->getEndEffectorLink();//将物体依附到机械臂末端
    collision_object4.touch_links = std::vector<std::string>{ group->getEndEffectorLink()};
    collision_object4.object.id = "boxRobot";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
        
    primitive.dimensions[0] = 0.05;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.03;

     geometry_msgs::Pose box_pose;
     box_pose.position.x = 0.06;
     box_pose.position.y = 0;
     box_pose.position.z = 0;
     box_pose.orientation.w = 1;

    collision_object4.object.primitives.push_back(primitive);
    collision_object4.object.primitive_poses.push_back(box_pose);

    collision_object4.object.operation = collision_object4.object.ADD;  

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.robot_state.attached_collision_objects.push_back(collision_object4);
    planning_scene_diff_publisher.publish(planning_scene);
    sleep(5.0);
    moveit_msgs::CollisionObject collision_object1 = createBox(Eigen::Vector3f(2,2,0.1),Eigen::Vector3f(0,0,-0.2));
    moveit_msgs::CollisionObject collision_object2 = createBox(Eigen::Vector3f(2,0.1,2),Eigen::Vector3f(0,-0.5,1));
    moveit_msgs::CollisionObject collision_object3 = createBox(Eigen::Vector3f(2,0.1,2),Eigen::Vector3f(0,0.5,1));

    moveit::planning_interface::PlanningSceneInterface current_scene;
    std::vector<moveit_msgs::CollisionObject> collision_objects = {collision_object1, collision_object2 ,collision_object3};
    current_scene.addCollisionObjects(collision_objects);
    QMessageBox::information(nullptr,"info","add collision objects finished !",QMessageBox::Ok);
}

void CURControl::planningPose(const Eigen::VectorXf& nextPose)
{
        geometry_msgs::Pose Specified_pose;
        Specified_pose.position.x = nextPose[0];
        Specified_pose.position.y = nextPose[1];
        Specified_pose.position.z = nextPose[2];
        Specified_pose.orientation.x =  nextPose[3];
        Specified_pose.orientation.y =  nextPose[4];
        Specified_pose.orientation.z =  nextPose[5];
        Specified_pose.orientation.w =  nextPose[6];

        group->setPoseTarget(Specified_pose);

        moveit::planning_interface::MoveGroupInterface::Plan planner;
    
	    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = 
		std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
	    monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
	    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
	    ps->getCurrentStateNonConst().update();
	    planning_scene::PlanningScenePtr scene = ps->diff();
    	scene->decoupleParent();
        
        auto is_success=group->plan(planner);
        if(is_success== moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            group->move();
            QMessageBox::information(nullptr,"info","plan successed !",QMessageBox::Ok);
        }
        else
        {
           QMessageBox::warning(nullptr,"waring","plan failed !",QMessageBox::Ok);
        }  
}

void CURControlThread::run(){
    ros::init(argc_,argv_,"my_UR_Control");
    ros::AsyncSpinner spinner(8); // Use8 threads  
    spinner.start();
    kl_ = new  CURControl();
    ros::waitForShutdown();       
}

void CURControlThread::slotGetPose(const Eigen::VectorXf& nextPose)
{
    kl_->planningPose(nextPose);
}

void CURControlThread::slotInitGroup()
{
    kl_->init();
}

void CURControlThread::slotAddCollision()
{
    kl_->addCollision();
}