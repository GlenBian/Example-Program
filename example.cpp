
/*
 * Copyright (C) 2018 Youibot Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// system
#include <iostream>
#include <fstream>
#include <stdio.h>
// ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
// opencv
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
// pcl
#include <pcl/sample_consensus/rmsac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <boost/foreach.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

using namespace std;
using namespace cv;
typedef pcl::PointXYZ PointT;

static const string OPENCV_WINDOW = "Image window";

class ImageConvert{

    typedef struct Box
    {
        double x;
        double y;
        double r;
    }Box;

    ros::NodeHandle nh;

    image_transport::ImageTransport it_;

    image_transport::Subscriber image_sub_;

    ros::Subscriber image_sub_1;

    ros::Subscriber image_sub_2;

    image_transport::Publisher image_pub_;

    ros::Publisher image_pub_1;

    ros::Publisher tyre_radius_pub;

    image_transport::Subscriber good_bad_pub_1;

    // 是否进行图像显示
    bool showImage = true;

    // 是否将图像的像素值保存下来
    bool txtImage = false;

    // 是否显示图像的直方图
    bool histImage = true;

    // 二值化阈值
    int threshold_ = 0;

    // 平面到机器人的距离
    double distance = 0;

    // 坏点point记录
    vector<Point2d> badPoints;

    int nY20_thresh = 110;

    Mat src;

    FILE *fp;

public:

    Mat dst,gbi,XImage,YImage;

/**
 * @brief  订阅O3d3摄像机的图像数据
 * @author GlenBian
 * @date   2018-04-10
*/
    ImageConvert()
        : it_(nh)
    {
        // 订阅和发布话题对象放在一起
        image_sub_ = it_.subscribe("/o3d3xx/camera/depth_viz",1,&ImageConvert::imageCbDepthViz,this);

        image_sub_2 = nh.subscribe("/o3d3xx/camera/xyz_image",1,&ImageConvert::imageCbDepth,this);

        image_sub_1 = nh.subscribe("/o3d3xx/camera/cloud",10,&ImageConvert::imageCbCloud,this);

        image_pub_ = it_.advertise("output_image_topic",1);

        image_pub_1 = nh.advertise<std_msgs::Float32>("dis_btw_bus_robot_topic",10);

        tyre_radius_pub = nh.advertise<std_msgs::Float32>("tyreRadius_topic",10);

        good_bad_pub_1 = it_.subscribe("/o3d3xx/camera/good_bad_pixels",1,&ImageConvert::goodbadCb,this);


        pcl::visualization::CloudViewer viewer ("Cloud");

        if(showImage){

            namedWindow(OPENCV_WINDOW);
        }
        if(txtImage)
        {

            fp = fopen("/home/youibot/imagevalue.txt","w");
        }
    }
    ~ImageConvert(){

        if(showImage){

            destroyWindow(OPENCV_WINDOW);
        }
        if(txtImage)
        {

            fclose(fp);
        }
    }
    
/**
 * @brief  订阅好坏点话题
 * @author GlenBian
 * @date   2018-04-16
*/
    void goodbadCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr_;
        try
        {
            string type = msg->encoding;

            // 图像类型为mono8
            cv_ptr_ = cv_bridge::toCvCopy(msg,type);
            gbi = cv_ptr_->image;
            Mat badImage = cv_ptr_->image;
            badPoints.clear();
            for(int i =0 ; i < badImage.rows;i++)
            {
                for(int j = 0 ; j < badImage.cols ; j++)
                {
                    if(badImage.at<uchar>(j,i) == 255)
                    {
                        badPoints.push_back(Point2d(j,i));
                    }
                }
            }
            cout<<badPoints.size()<<endl;
        }catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s",e.what());
            return;
        }
    }
/**
 * @brief  image_sub_的回调函数，进行opencv处理
 * @author GlenBian
 * @date   2018-03-10
*/
    void imageCbCloud(const sensor_msgs::PointCloud2Ptr& msg)
    {
        int width = msg->width;
        int height = msg->height;
        // Length of a row in bytes
        int row_steps = msg->row_step;
        // Length of a point in bytes
        int point_steps = msg->point_step;
        pcl::PCLPointCloud2 pcl_pc2;
        cloud = new pcl::PointCloud<pcl::PointXYZ>();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_conversions::toPCL(*msg,pcl_pc2);
        // 点云中的点的数量为23232
        pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

        /****  Mandatory 设置目标几何形状
         * SACMODEL_PLANE 特征是ax+by+cz+d = 0，如果d = 0,表示平面过坐标系原点；
         * SACMODEL_SPHERE values大小是4，position.x,position.y,position.z,radius
         * SACMODEL_CIRCLE2D 检测在一个平面内的2D圆，[center.x,center.y,radius]
         * SACMODEL_CYLINDER 检测圆柱,[point_on_axis.x,point_on_axis.y,point_on_axis.z,axis_direction.x,axis_direction.y,axis_direction.z,redius]
         * SACMODEL_LINE [point_on_line.x,point_on_line.y,point_on_line.z,line_direction.x,line_direction.y,line_direction.z]
         ****/
        pcl::NormalEstimation<PointT,pcl::Normal> ne; // 法线估计对象
        pcl::SACSegmentationFromNormals<PointT,pcl::Normal> seg;
        pcl::PCDWriter writer;
        pcl::ExtractIndices<PointT> extract;
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
        pcl::PassThrough<PointT> pass; // 直通滤波器

        // database
        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_ (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered3 (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals3 (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals4 (new pcl::PointCloud<pcl::Normal>);
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_circle (new pcl::ModelCoefficients),coefficients_cylinder (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices),inliers_circle (new pcl::PointIndices);

        // 直通滤波，将Z轴不在（0，1.5）范围的点过滤掉，将剩余的点存储到cloud_filtered对象中 此处的z轴为竖直方向，深度为x轴
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (0,0.8);
        pass.filter (*cloud_filtered);

        // 过滤后的点云进行法线估计，为后续进行基于法线的分割准备数据
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud_filtered);
        ne.setKSearch (20);
        ne.compute (*cloud_normals);
        std::cerr << "PointCloud representing the normal component: " << cloud_normals->points.size() << " normal points." << std::endl;

        // Create the segmentation object for the planar model and set all the parameters
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setNormalDistanceWeight (0.1);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.025);
        seg.setInputCloud (cloud_filtered);
        seg.setInputNormals (cloud_normals);
        //获取平面模型的系数和处在平面的内点
        seg.segment (*inliers_plane, *coefficients_plane);
//        std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
        // 从点云中抽取分割的处在平面上的点集
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers_plane);
        // 保留平面局内点,移除剩余点云
        extract.setNegative (false);

        // 存储分割得到的平面上的点到点云文件
        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
        extract.filter (*cloud_plane);

        // Remove the planar inliers, extract the rest
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers_plane);
        extract.setNegative (true);
        extract.filter (*cloud_filtered2);
        extract_normals.setNegative (true);
        extract_normals.setInputCloud (cloud_normals);
        extract_normals.setIndices (inliers_plane);
        extract_normals.filter (*cloud_normals2);
        std::cerr<<"PointCloud rest of plane has saved!"<<cloud_filtered2->points.size()<<std::endl;
        writer.write ("rest_plane_.pcd", *cloud_filtered2, false);

        std_msgs::Float32 dis_msgs;

        distance = coefficients_plane->values[3];

        dis_msgs.data = coefficients_plane->values[3];

        image_pub_1.publish(dis_msgs);
    }

    void imageCbDepth(const sensor_msgs::ImageConstPtr& msg1)
    {
        cv_bridge::CvImagePtr cv_ptr_;
        try
        {
            string type = msg1->encoding;
            // cout<<type<<endl;
            // 图像类型为16SC3
            cv_ptr_ = cv_bridge::toCvCopy(msg1,type);
            Mat xyz_image = cv_ptr_->image;
            // 根据深度图像进行二值化
            int width = xyz_image.cols;
            int height = xyz_image.rows;
            Mat mv[3];
            Mat xImage = Mat::zeros(Size(width,height),CV_16UC1);
            Mat yImage = Mat::zeros(Size(width,height),CV_16UC1);
            Mat zImage = Mat::zeros(Size(width,height),CV_16UC1);
            split(xyz_image,mv);
            zImage = mv[0];
            xImage = mv[1];
            yImage = mv[2];
            XImage = xImage;
            YImage = yImage;
            // 获得x方向的距离值 实验证明x方向的分辨率在4mm，y方向的分辨率在2.4mm
            double x_left = 0;
            double x_right = 0;
            double x_sum = 0;
            for(int i = 0 ; i < height ; i++)
            {
                x_sum = x_sum + xImage.at<short>(i,0);
            }
            x_left = x_sum / height;
            double x_sum_ = 0;
            for(int i = 0 ; i < height ; i++)
            {
                x_sum_ = x_sum_ + xImage.at<short>(i,width-1);
            }
            x_right = x_sum_ / height;
            double width_x = abs(x_right - x_left);
            cout<<"x方向的宽度为："<<width_x<<endl;
            double y_left = 0;
            double y_right = 0;
            double y_sum = 0;
            for(int i = 0 ; i < width ; i++)
            {
                y_sum = y_sum + yImage.at<short>(0,i);
            }
            y_left = y_sum / width;
            double y_sum_ = 0;
            for(int i = 0 ; i < width ; i++)
            {
                y_sum_ = y_sum_ + yImage.at<short>(height-1,i);
            }
            y_right = y_sum_ / width;
            double width_y = abs(y_right - y_left);
            cout<<"y方向的宽度为："<<width_y<<endl;

            Mat binaryImage1 = Mat::zeros(Size(width,height),CV_8UC1);
            for(int i = 0;i < height;i++)
                for(int j = 0;j < width;j++)
                {
                    // 判断距离是否大于400mm
                    if(abs(zImage.at<short>(i,j)) > abs(distance) * 1000 * 1.15 )
                    {
                        binaryImage1.at<uchar>(i,j) = 255;
                    }else{
                        binaryImage1.at<uchar>(i,j) = 0;
                    }
                }

            Mat cannyImage;
            dst = binaryImage1;
            // canny
            Canny(binaryImage1,cannyImage,50,150);

            if(txtImage)
            {
                for(int i = 0;i<xImage.rows;i++)
                {
                    for(int j = 0;j < xImage.cols; j++)
                    {
                        fprintf(fp,"%d ",xImage.at<short>(i,j));
                    }
                    fprintf(fp,"\n");
                }
                fprintf(fp,"\n new \n");
            }
        }catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s",e.what());
            return;
        }
    }

    /**
 * @brief  提取ransac特征进行提取，输入图像为cannny边缘检测后的图像
 * @author GlenBian
 * @date   2018-04-13
*/
    void imageCbDepthViz(const sensor_msgs::ImageConstPtr& msg){

        Point p_click;
//        sensor_msgs::Image _ros_image = msg->data;
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");

            src = cv_ptr->image;
            imwrite("/home/youibot/bgrsift1.jpg",src);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s",e.what());
            return;
        }
        // 灰度化处理
        cvtColor(src,src,CV_RGB2GRAY);
        Mat image = src;
        int channels = 0;
        int histSize[] = {256};
        float midRanges[] = {0,256};
        const float *ranges[] = {midRanges};
        MatND hist;
        calcHist(&image,1,&channels,Mat(),hist,1,histSize,ranges,true,false);
        if(histImage)
        {
            // 显示直方图
            Mat drawImage = Mat::zeros(Size(256,256),CV_8UC3);
            // 任何一个图像的某个像素的总个数有可能会很多，甚至超过所定义的图像的尺寸
            double g_dHistMaxValue;
            minMaxLoc(hist,0,&g_dHistMaxValue,0,0);
            // 将图像的像素整合到图像的最大范围内
            for(int i = 0 ; i < 256 ; i++)
            {
                int value = cvRound(hist.at<float>(i) * 256 * 0.9 / g_dHistMaxValue);

                line(drawImage,Point(i,drawImage.rows - 1),Point(i,drawImage.rows - 1 - value),Scalar(0,0,255));
            }
            imshow("hist",drawImage);
            waitKey(3);
        }
        // 获得二值化阈值
        threshold_ = GetOSTUThreshold(hist);
        Mat binaryImage = Mat::zeros(src.size(),CV_8UC1);
        int width = src.cols;
        int height = src.rows;
        for(int i = 0 ; i < height ; i++)
        {
            for(int j = 0 ; j < width ; j++)
            {
                if(src.at<uchar>(i,j) >= threshold_)
                {
                    binaryImage.at<uchar>(i,j) = 255;
                }else{
                    binaryImage.at<uchar>(i,j) = 0;
                }
            }
        }
        // 进行边界扩充
//        Mat bigImage;
//        copyMakeBorder(binaryImage,bigImage,0,height * 3,width,width,BORDER_CONSTANT,Scalar(255));
        // 高斯滤波
        Mat blurImage;
        GaussianBlur(binaryImage,blurImage,Size(3,3),1);
        Mat cannyImage;
        // canny
        Canny(blurImage,cannyImage,50,150);

        imwrite("/home/youibot/depth_viz.jpg",cannyImage);
        // 移除竖线
        int col_left = width - 1;
        for(int i = 0 ; i < height ; i++)
        {
            cannyImage.at<uchar>(i,col_left) = 0;
        }
        int col_right = width * 2 -1;
        for(int i = 0 ; i < height ; i++)
        {
            cannyImage.at<uchar>(i,col_right) = 0;
        }
        if(showImage)
        {
            imshow(OPENCV_WINDOW,blurImage);
            waitKey(3);
        }
        if(txtImage)
        {
            for(int i = 0;i<src.rows;i++)
            {
                for(int j = 0;j < src.cols; j++)
                {
                    fprintf(fp,"%d ",src.at<int>(i,j));
                }
                fprintf(fp,"\n");
            }
            fprintf(fp,"\n");
        }
//        ROS_INFO("The center value is %d",src.at<uchar>(dst.rows/2,dst.cols/2));
        image_pub_.publish(cv_ptr->toImageMsg());
    }
    /**
 * @brief  获得OSTU最佳阈值
 * @author GlenBian
 * @date   2018-04-11
*/
    static int GetOSTUThreshold(MatND hist){

        int X,Y,Amount = 0;
        int PixelBack = 0,PixelFore = 0,PixelIntergralBack = 0,PixelIntegralFore = 0,PixelIntegral = 0;
        double OmegaBack,OmegaFore,MicroBack,MicroFore,SigmaB,Sigma; // 类间方差
        int MinValue,MaxValue;
        int Threshold = 0;

        for(MinValue = 0;MinValue < 256 && hist.at<float>(MinValue) == 0;MinValue++);
        for(MaxValue = 255;MaxValue > MinValue && hist.at<float>(MinValue) == 0;MaxValue--);
        if(MaxValue == MinValue) return MaxValue; // 图像只有一种颜色
        if(MinValue + 1 == MaxValue) return MinValue; //图像只有两种颜色

        for(Y = MinValue ; Y <= MaxValue ; Y++) Amount += hist.at<float>(Y); // 像素总数

        PixelIntegral = 0;
        for(Y = MinValue ; Y <= MaxValue ; Y++) PixelIntegral += hist.at<float>(Y) * Y;
        SigmaB = -1;
        for (Y = MinValue ; Y < MaxValue ; Y++)
        {
            PixelBack = PixelBack + hist.at<float>(Y);
            PixelFore = Amount - PixelBack;
            OmegaBack = (double)PixelBack / Amount;
            OmegaFore = (double)PixelFore / Amount;
            PixelIntergralBack += hist.at<float>(Y) * Y;
            PixelIntegralFore = PixelIntegral - PixelIntergralBack;
            MicroBack = (double)PixelIntergralBack / PixelBack;
            MicroFore = (double)PixelIntegralFore / PixelFore;
            Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
            if(Sigma > SigmaB)
            {
                SigmaB = Sigma;
                Threshold = Y;
            }
        }

        return Threshold;
    }

    /**
 * @brief  RANSAC  判断是否是圆
 * @author GlenBian
 * @date   2018-04-19
*/
    float verifyCircle(Mat dt, Point2f center, float radius, vector<Point2f> & inlierSet)
    {
     unsigned int counter = 0;
     unsigned int inlier = 0;
     float minInlierDist = 2.0f;
     float maxInlierDistMax = 100.0f;
     float maxInlierDist = radius/70.0f;
    // float maxInlierDist = 5.0f;
     if(maxInlierDist<minInlierDist) maxInlierDist = minInlierDist;
     if(maxInlierDist>maxInlierDistMax) maxInlierDist = maxInlierDistMax;

     // choose samples along the circle and count inlier percentage
     for(float t =0; t<2*3.14159265359f; t+= 0.01f)
     {
         counter++;
         float cX = radius*cos(t) + center.x;
         float cY = radius*sin(t) + center.y;

         if(cX < dt.cols)
         if(cX >= 0)
         if(cY < dt.rows)
         if(cY >= 0)
         if(dt.at<float>(cY,cX) < maxInlierDist)
         {
            inlier++;
            inlierSet.push_back(Point2f(cX,cY));
         }
     }

     return (float)inlier/float(counter);
    }
    /**
 * @brief  根据随机选取的三个点来计算圆心和半径
 * @author GlenBian
 * @date   2018-04-19
*/
     void getCircle(Point2f& p1,Point2f& p2,Point2f& p3, Point2f& center, float& radius)
    {
      float x1 = p1.x;
      float x2 = p2.x;
      float x3 = p3.x;

      float y1 = p1.y;
      float y2 = p2.y;
      float y3 = p3.y;

      // PLEASE CHECK FOR TYPOS IN THE FORMULA :)
      center.x = (x1*x1+y1*y1)*(y2-y3) + (x2*x2+y2*y2)*(y3-y1) + (x3*x3+y3*y3)*(y1-y2);
      center.x /= ( 2*(x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2) );

      center.y = (x1*x1 + y1*y1)*(x3-x2) + (x2*x2+y2*y2)*(x1-x3) + (x3*x3 + y3*y3)*(x2-x1);
      center.y /= ( 2*(x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2) );

      radius = sqrt((center.x-x1)*(center.x-x1) + (center.y-y1)*(center.y-y1));


    }
     /**
 * @brief  由二值化图片得到点向量
 * @author GlenBian
 * @date   2018-04-19
*/
    vector<Point2f> getPointPositions(Mat binaryImage)
    {
     vector<Point2f> pointPositions;

     for(unsigned int y=0; y<binaryImage.rows; ++y)
     {

         for(unsigned int x=0; x<binaryImage.cols; ++x)
         {

             if(binaryImage.at<unsigned char>(y,x) > 0) pointPositions.push_back(Point2f(x,y));
         }
     }

     return pointPositions;
    }
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"image_converter");
    ImageConvert ic;
    ros::NodeHandle hhh;
    ros::Rate loop_rate(10);
    int width = 176;
    int height = 132;
    vector<Point2d> leftPoints;
    vector<Point2d> rightPoints;
    bool beginLoc = false;
    bool beginRansac = false;
    while(ros::ok())
    {
        ros::spinOnce();
        Mat showImage(height,width,CV_8UC1,Scalar(0));
        Mat image(132,176,CV_8UC1);
        // 还原真实图像
        if(ic.dst.rows > 0 && ic.gbi.rows > 0)
        {
            cout<<ic.dst.rows<<" "<<ic.dst.cols<<" "<<ic.gbi.rows<<" "<<ic.gbi.cols<<endl;
            for(int i = 0 ; i<height ; i++)
            {
                for(int j = 0 ; j<width ; j++)
                {
                    int pixel_value = ic.dst.at<uchar>(i,j) + ic.gbi.at<uchar>(i,j);
                    if(pixel_value > 255)
                    {
                        pixel_value = 255;
                    }
                    image.at<uchar>(i,j) = pixel_value;
                }
            }
        }
        Mat cannyImage;
        Canny(image,cannyImage,50,150);
        // 获得左右边缘点的数量
        int left_count = 0 ;
        int right_count = 0 ;
        leftPoints.clear();
        rightPoints.clear();
        for(int i = 0 ; i < height ; i++)
        {
            if(cannyImage.at<uchar>(i,0) == 255)
            {
                leftPoints.push_back(Point2d(i,0));
            }
        }
        for(int i = 0 ; i < height ; i++)
        {
            if(cannyImage.at<uchar>(i,width - 1) == 255)
            {
                rightPoints.push_back(Point2d(i,width - 1));
            }
        }

        imshow("Final Image",cannyImage);
        waitKey(3);

        cout<<"左边界的点数"<<leftPoints.size()<<"右边界的点数"<<rightPoints.size()<<endl;

        if(leftPoints.size() >= 2 && rightPoints.size() >= 2)
        {

            beginLoc = true;

        }else{

            beginLoc = false;

            cout<<"未到位"<<endl;

        }
        // 判断是否进行ransac圆弧识别
        if(beginLoc)
        {
            Mat mask = cannyImage; //for the rest of the code

            vector<Point2f> edgePositions;
            edgePositions = ic.getPointPositions(mask);

            Mat dt;
            // 像素按位取反
            mask = ~mask;
            // dt 中每个像素点的值是非零像素点到其最近的零像素点的距离 distance = |x1-x2| + |y1-y2|
            distanceTransform(mask, dt,CV_DIST_L1, 3);

            unsigned int nIterations = 0;

            Point2f bestCircleCenter;
            float bestCircleRadius;
            float bestCirclePercentage = 0;
            float minRadius = 200; //can change for smaller radius
            //float minCirclePercentage = 0.2f;
            float minCirclePercentage = 0.1f;  // at least 5% of a circle must be present? maybe more...

            int maxNrOfIterations = edgePositions.size();

            for(unsigned int its=0; its< maxNrOfIterations; ++its)
            {
                //RANSAC: randomly choose 3 point and create a circle:
                //TODO: choose randomly but more intelligent,
                //so that it is more likely to choose three points of a circle.
                //For example if there are many small circles, it is unlikely to randomly choose 3 points of the same circle.
                unsigned int idx1 = rand()%edgePositions.size();
                unsigned int idx2 = rand()%edgePositions.size();
                unsigned int idx3 = rand()%edgePositions.size();

                // we need 3 different samples:
                if(idx1 == idx2) continue;
                if(idx1 == idx3) continue;
                if(idx3 == idx2) continue;

                // create circle from 3 points:
                Point2f center; float radius;
                ic.getCircle(edgePositions[idx1],edgePositions[idx2],edgePositions[idx3],center,radius);

                // inlier set unused at the moment but could be used to approximate a (more robust) circle from alle inlier
                vector<Point2f> inlierSet;

                //verify or falsify the circle by inlier counting:
                float cPerc = ic.verifyCircle(dt,center,radius, inlierSet);

                // update best circle information if necessary
                if(cPerc >= bestCirclePercentage)
                    if(radius >= minRadius)
                {
                    bestCirclePercentage = cPerc;
                    bestCircleRadius = radius;
                    bestCircleCenter = center;
                }
            }

            cout<<"Best"<<bestCirclePercentage<<"Center"<<bestCircleCenter<<endl;

            if(bestCirclePercentage >= minCirclePercentage)
            {
                if(bestCircleRadius >= minRadius)
                {
                    circle(showImage, bestCircleCenter,bestCircleRadius, Scalar(255),1);
                    cout<< "The radius for this part is "<<bestCircleRadius<<"\n";
                    imshow("output",showImage);
                    waitKey(3); //fr dispaly time.
                }else{
                    cout<<"半径不满足要求"<<endl;
                }
            }else{
                cout<<"没找到合适的圆弧"<<endl;
            }

            Point2d roofPoint;
            int tyreH;
            int tyreLeft_Position = 0;
            int tyreRight_Position = 0;

            for(int i = 0 ; i < height ; i++)
            {
                if(showImage.at<uchar>(i,0) == 255)
                {
                    tyreLeft_Position = i;
                }
            }
            for(int i = 0 ; i < height ; i++)
            {
                if(showImage.at<uchar>(i,width-1) == 255)
                {
                    tyreRight_Position = i;
                }
            }

//            int tyreLeft_Position = leftPoints[leftPoints.size() - 1].x;
//            int tyreRight_Position = rightPoints[rightPoints.size() - 1].x;
            cout<<"左边最下位置为："<<tyreLeft_Position<<"右边最下位置为："<<tyreRight_Position<<endl;
            if(abs(tyreLeft_Position - tyreRight_Position) < 2)
            {
                cout<<"到达中心"<<endl;

                for(int i = 0 ; i < height ; i++)
                {
                    if(showImage.at<uchar>(i,width / 2) == 255)
                    {
                        tyreH = i;
                    }
                }

                cout<<tyreH<<endl;

                double y_sum = 0;

                double y_value = 0;

                for(int i = 0 ; i < width;i++)
                {
                    y_sum = y_sum + ic.YImage.at<short>(tyreH,i);
                }

                y_value = y_sum / width;

                cout<<"半径是"<<y_value<<endl;

            }else{
                cout<<"发布速度命令"<<endl;
            }
        }


        loop_rate.sleep();
    }

    return 0;
}
