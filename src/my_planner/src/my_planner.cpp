#include "my_planner.h"
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

PLUGINLIB_EXPORT_CLASS( my_planner::MyPlanner, nav_core::BaseLocalPlanner)

// PID控制参数定义
double Kp = 2.0;  // 比例系数1.4
double Ki = 0.0;  // 积分系数
double Kd = 1.0;  // 微分系数1.1

// PID控制所需变量
double angular_error = 0;   // 当前误差
double last_error = 0;      // 上一次误差
double error_sum = 0;       // 误差累积
double error_diff = 0;      // 误差变化率
double output = 0;          // PID输出值

namespace my_planner 
{
    MyPlanner::MyPlanner()
    {
        setlocale(LC_ALL,"");
    }
    MyPlanner::~MyPlanner()
    {}

    tf::TransformListener* tf_listener_;
    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("该我上场表演了！");
        tf_listener_ = new tf::TransformListener();   // 创建TF监听器
    }

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    int target_index_;
    bool pose_adjusting_;
    bool goal_reached_;
    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        target_index_ = 0;         // 重置目标点索引
        global_plan_ = plan;       // 存储全局路径
        pose_adjusting_ = false;   // 重置姿态调整标志
        goal_reached_ = false;     // 重置到达标志
        return true;
    }

    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        int final_index = global_plan_.size()-1;
        geometry_msgs::PoseStamped pose_final;
        global_plan_[final_index].header.stamp = ros::Time(0);
        // 获取终点在机器人坐标系中的位置
        tf_listener_->transformPose("tianracer/base_link",global_plan_[final_index],pose_final);
        if(pose_adjusting_ == false)
        {   
            // 计算与终点的距离
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            // 距离<1m时进入姿态调整模式
            if(dist < 1.5)
                pose_adjusting_ = true;
        }
        // 在姿态调整模式下标记到达终点
        if(pose_adjusting_ == true)
        {
            goal_reached_ = true;
            ROS_WARN("到达终点！");
            return true;
        }
        0
        geometry_msgs::PoseStamped target_pose;
        // 遍历路径点（从当前索引开始）
        for(int i=target_index_;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            // 转换到机器人坐标系
            tf_listener_->transformPose("tianracer/base_link",global_plan_[i],pose_base);
            // 计算距离
            double dx = pose_base.pose.position.x;
            double dy = pose_base.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist > 0.8) //将挑选目标路径点的距离阈值设置为0.8米
            {
                target_pose = pose_base;
                target_index_ = i;
                ROS_WARN("选择第 %d 个路径点作为临时目标，距离=%.2f",target_index_,dist);
                break;
            }

            if(i == global_plan_.size()-1)
                target_pose = pose_base; 
        }
        // 线速度：与目标点X坐标成正比
        cmd_vel.linear.x = target_pose.pose.position.x * 4.6;//4.2
        
        // PID控制器计算角速度
        // 计算误差
        angular_error = target_pose.pose.position.y;  // 目标值和当前值的差  横向偏差=Y坐标
        // 计算积分项
        error_sum += angular_error ;
        // 计算微分项
        error_diff = angular_error - last_error;
        // 计算PID控制器最终输出
        output = Kp * angular_error + Ki * error_sum + Kd * error_diff;
        // 角速度使用PID最终输出
        cmd_vel.angular.z = output;
        // 记录误差数值
        last_error = angular_error;

        // cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));        
        // for(int i=0;i<global_plan_.size();i++)
        // {
        //     geometry_msgs::PoseStamped pose_base;
        //     global_plan_[i].header.stamp = ros::Time(0);
        //     tf_listener_->transformPose("tianracer/base_link",global_plan_[i],pose_base);
        //     int cv_x = 300 - pose_base.pose.position.y*100;
        //     int cv_y = 300 - pose_base.pose.position.x*100;
        //     cv::circle(plan_image, cv::Point(cv_x,cv_y), 1, cv::Scalar(255,0,255)); 
        // }
        // cv::circle(plan_image, cv::Point(300, 300), 15, cv::Scalar(0, 255, 0));
        // cv::line(plan_image, cv::Point(65, 300), cv::Point(510, 300), cv::Scalar(0, 255, 0), 1);
        // cv::line(plan_image, cv::Point(300, 45), cv::Point(300, 555), cv::Scalar(0, 255, 0), 1);

        // cv::namedWindow("Plan");
        // cv::imshow("Plan", plan_image);
        // cv::waitKey(1);
        
        return true;
    }
    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    } 
} // namespace my_planner
