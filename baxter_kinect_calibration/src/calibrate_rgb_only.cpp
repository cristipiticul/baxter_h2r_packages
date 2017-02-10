#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ar_track_alvar/MarkerDetector.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <set>

double max_new_marker_error = 0.08; // same value as in ar_track_alvar/launch/pr2_indiv_no_kinect.launch
double max_track_error = 0.2; // same value as in ar_track_alvar/launch/pr2_indiv_no_kinect.launch
double marker_size = 8.0;
const char calibratedCameraFrame[] = "left_hand_camera";
const char robotLinkForCalibration[] = "left_wrist";

tf::TransformListener* pTransformListener;
tf::TransformBroadcaster* pTransformBroadcaster;

ros::Publisher pointCloudPublisher;

alvar::MarkerDetector<alvar::MarkerData> calibratedDetector;
alvar::Camera* calibratedCam;
cv::Mat calibratedCameraImage;
cv::Mat noncalibratedCameraImage;
volatile bool calibratedCameraCallbackEnabled = false;
volatile bool calibratedCameraDetectionDone = false;
volatile bool noncalibratedCameraCallbackEnabled = false;
volatile bool noncalibratedCameraDetectionDone = false;

void calibratedCameraCallback(const sensor_msgs::ImageConstPtr & image_msg);
void noncalibratedCameraCallback(const sensor_msgs::ImageConstPtr & image_msg);
tf::Transform getTransformFromPose(alvar::Pose &p);
bool getTransformFromListener(const char from[], const char to[],
		tf::StampedTransform &result);
void detect();

class SimpleDetector
{
private:
	ros::NodeHandle n;
	image_transport::ImageTransport it;

	alvar::MarkerDetector<alvar::MarkerData> detector;
	std::string cameraInfoTopic;
	alvar::Camera cameraInfo;
	cv::Mat lastImage;
	image_transport::Subscriber imageSubscriber;

	std::string cameraFrame;
	std::string calibrationFrame;
	tf::StampedTransform calibrationFrameToCameraFrame;

	volatile bool callbackEnabled;
	volatile bool detectionDone;

	int imageHeight;
	int imageWidth;
public:
	SimpleDetector(std::string cameraInfoTopicParam, std::string imageTopic,
			std::string cameraFrameParam, std::string calibrationFrameParam,
			int imageHeightParam, int imageWidthParam) :
			n(),
			it(n),
			cameraInfoTopic(cameraInfoTopicParam),
			cameraInfo(n, cameraInfoTopicParam),
			imageSubscriber(it.subscribe(imageTopic, 1, &SimpleDetector::callback, this)),
			cameraFrame(cameraFrameParam),
			calibrationFrame(calibrationFrameParam),
			callbackEnabled(false),
			detectionDone(false),
			imageHeight(imageHeightParam),
			imageWidth(imageWidthParam)
	{
		detector.SetMarkerSize(marker_size);
	}

	void callback(const sensor_msgs::ImageConstPtr & image_msg);
	void startDetection();
	void waitForDetectionToFinish();
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "marker_detect_rgb_only");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	std::string calibratedImageTopic = "/cameras/left_hand_camera/image";
	std::string calibratedInfoTopic = "/cameras/left_hand_camera/camera_info";
	std::string uncalibratedImageTopic = "/camera/rgb/image_raw";
	std::string uncalibratedInfoTopic = "/camera/rgb/camera_info";
	std::string uncalibratedCameraFrame = "camera_rgb_optical_frame";
	std::string uncalibratedCameraCalibrationFrame = "camera_link";

	image_transport::Subscriber calibratedSub_;
	image_transport::Subscriber uncalibratedSub_;

	pTransformListener = new tf::TransformListener(n);
	pTransformBroadcaster = new tf::TransformBroadcaster();
	pointCloudPublisher = n.advertise<sensor_msgs::PointCloud2>(
			"calibration_debug_point_cloud", 1);

	// Give transform listener time to get some data.
	ros::Duration(1.0).sleep();

	calibratedCam = new alvar::Camera(n, calibratedInfoTopic);

	image_transport::ImageTransport it_(n);
	//Subscribe to topics and set up callbacks
	SimpleDetector simpleDetector(uncalibratedInfoTopic, uncalibratedImageTopic, uncalibratedCameraFrame, uncalibratedCameraCalibrationFrame, 480, 640);
	simpleDetector.startDetection();
	simpleDetector.waitForDetectionToFinish();
	/*
	ROS_INFO_STREAM(
			"Subscribing to calibrated image topic '" << calibratedImageTopic << "'");
	calibratedSub_ = it_.subscribe(calibratedImageTopic, 1,
			&calibratedCameraCallback);
*/
	/*
	 ROS_INFO_STREAM(
	 "Subscribing to uncalibrated image topic '" << uncalibratedImageTopic << "'");
	 uncalibratedSub_ = it_.subscribe(uncalibratedImageTopic, 1,
	 &getCapCallbackWithMonitor);
	 */
	/*
	char done;
	do
	{
		std::cout << "done? ";
		std::cin >> done;
		if (done != 'y')
		{
			detect();
		}
	} while (done != 'y');
	while (ros::ok())
	{
		ros::Duration(0.1).sleep();
	}
	*/
	delete pTransformListener;
	delete pTransformBroadcaster;
	return 0;
}

void SimpleDetector::callback(const sensor_msgs::ImageConstPtr & image_msg)
{
	if (callbackEnabled)
	{
		if (detectionDone)
		{
			ROS_WARN("You forgot to clear the detectionDone flag!");
			detectionDone = false;
		}
		callbackEnabled = false;

		//If no camera info, return
		if (!cameraInfo.getCamInfo_)
		{
			ROS_WARN("No camera info on topic %s", cameraInfoTopic.c_str());
			return;
		}

		cv_bridge::CvImagePtr cv_ptr_;
		try
		{
			cv_ptr_ = cv_bridge::toCvCopy(image_msg,
					sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to 'rgb8'.",
					image_msg->encoding.c_str());
		}

		//Get the estimated pose of the main markers by using all the markers in each bundle
		IplImage ipl_image = cv_ptr_->image;
		detector.Detect(&ipl_image, &cameraInfo, true, true,
				max_new_marker_error, max_track_error, alvar::CVSEQ, true);
		lastImage = cv_ptr_->image.clone();

		if (ipl_image.height != imageHeight || ipl_image.width != imageWidth)
		{
			ROS_ERROR(
					"Wrist camera image is incorrect size! Should be 1280x800. Shutting down.");
			exit(1);
		}
		detectionDone = true;
	}
}

void SimpleDetector::startDetection()
{
	bool transformIsRecent = getTransformFromListener(calibrationFrame.c_str(),
			cameraFrame.c_str(), calibrationFrameToCameraFrame);
	if (!transformIsRecent)
	{
		ROS_ERROR(
				"Transform calibrated_camera->robot_link was not retrieved successfully");
		return;
	}
	detectionDone = false;
	callbackEnabled = true;
}

void SimpleDetector::waitForDetectionToFinish()
{
	do
	{
		ros::Duration(0.1).sleep();
	}
	while (!detectionDone);

	cv::imshow("calibrated_camera_image", lastImage);
	cv::waitKey(0);
}

void noncalibratedCameraCallback(const sensor_msgs::ImageConstPtr & image_msg)
{
	if (noncalibratedCameraCallbackEnabled)
	{
		noncalibratedCameraCallbackEnabled = false;

		//If no camera info, return
		if (!calibratedCam->getCamInfo_)
		{
			return;
		}

		cv_bridge::CvImagePtr cv_ptr_;
		try
		{
			cv_ptr_ = cv_bridge::toCvCopy(image_msg,
					sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to 'rgb8'.",
					image_msg->encoding.c_str());
		}

		//Get the estimated pose of the main markers by using all the markers in each bundle
		IplImage ipl_image = cv_ptr_->image;
		calibratedDetector.Detect(&ipl_image, calibratedCam, true, true,
				max_new_marker_error, max_track_error, alvar::CVSEQ, true);
		noncalibratedCameraImage = cv_ptr_->image.clone();
		noncalibratedCameraDetectionDone = true;
	}
}

void detect()
{
	tf::StampedTransform robotLinkToCalibratedCamera;
	bool transformIsRecent = getTransformFromListener(robotLinkForCalibration,
			calibratedCameraFrame, robotLinkToCalibratedCamera);
	if (!transformIsRecent)
	{
		ROS_ERROR(
				"Transform calibrated_camera->robot_link was not retrieved successfully");
		return;
	}

	calibratedCameraDetectionDone = false;

	calibratedCameraCallbackEnabled = true;

	while (!calibratedCameraDetectionDone)
	{
		ros::Duration(0.1).sleep();
	}
	cv::imshow("calibrated_camera_image", calibratedCameraImage);
	cv::waitKey(0);

	ROS_INFO("Detected markers: %d", calibratedDetector.markers->size());
	for (int i = 0; i < calibratedDetector.markers->size(); i++)
	{
		alvar::Marker &marker = calibratedDetector.markers->at(i);
		ROS_INFO("Marker %d: %.3f %.3f %.3f", marker.GetId(),
				marker.pose.translation[0], marker.pose.translation[1],
				marker.pose.translation[2]);

		tf::Transform cameraToMarker = getTransformFromPose(marker.pose);
		tf::Transform robotLinkToMarker = robotLinkToCalibratedCamera
				* cameraToMarker;

		tf::Vector3 p1(-marker_size / 400.0, -marker_size / 400.0, 0.0);
		tf::Vector3 p1InRobotLinkFrame = robotLinkToMarker * p1;
		tf::Vector3 p2(-marker_size / 400.0, marker_size / 400.0, 0.0);
		tf::Vector3 p2InRobotLinkFrame = robotLinkToMarker * p2;
		tf::Vector3 p3(marker_size / 400.0, marker_size / 400.0, 0.0);
		tf::Vector3 p3InRobotLinkFrame = robotLinkToMarker * p3;
		tf::Vector3 p4(marker_size / 400.0, -marker_size / 400.0, 0.0);
		tf::Vector3 p4InRobotLinkFrame = robotLinkToMarker * p4;

		pcl::PointCloud<pcl::PointXYZ> pointCloud;
		sensor_msgs::PointCloud2 pointCloudRos;
		pointCloud.push_back(
				pcl::PointXYZ(p1InRobotLinkFrame.x(), p1InRobotLinkFrame.y(),
						p1InRobotLinkFrame.z()));
		pointCloud.push_back(
				pcl::PointXYZ(p2InRobotLinkFrame.x(), p2InRobotLinkFrame.y(),
						p2InRobotLinkFrame.z()));
		pointCloud.push_back(
				pcl::PointXYZ(p3InRobotLinkFrame.x(), p3InRobotLinkFrame.y(),
						p3InRobotLinkFrame.z()));
		pointCloud.push_back(
				pcl::PointXYZ(p4InRobotLinkFrame.x(), p4InRobotLinkFrame.y(),
						p4InRobotLinkFrame.z()));
		pcl::toROSMsg(pointCloud, pointCloudRos);
		pointCloudRos.header.frame_id = robotLinkForCalibration;
		pointCloudRos.header.stamp = ros::Time::now();
		pointCloudPublisher.publish(pointCloudRos);

		std::stringstream markerStr;
		markerStr << "marker_" << marker.GetId();

		tf::StampedTransform cameraToMarkerStamped(cameraToMarker,
				ros::Time::now(), calibratedCameraFrame,
				markerStr.str().c_str());
		pTransformBroadcaster->sendTransform(cameraToMarkerStamped);
	}
}

bool getTransformFromListener(const char from[], const char to[],
		tf::StampedTransform &result)
{
	tf::TransformListener listener;
	bool success = true;
	try
	{
		ros::Time now = ros::Time(0);
		pTransformListener->waitForTransform(from, to, now, ros::Duration(0.1));
		pTransformListener->lookupTransform(from, to, now, result);
	} catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
		success = false;
	}

	return success;
}

tf::Transform getTransformFromPose(alvar::Pose &p)
{
	double px, py, pz, qx, qy, qz, qw;

	px = p.translation[0] / 100.0;
	py = p.translation[1] / 100.0;
	pz = p.translation[2] / 100.0;
	qx = p.quaternion[1];
	qy = p.quaternion[2];
	qz = p.quaternion[3];
	qw = p.quaternion[0];

	//Get the marker pose in the camera frame
	tf::Quaternion rotation(qx, qy, qz, qw);
	tf::Vector3 origin(px, py, pz);
	tf::Transform t(rotation, origin);  //transform from cam to marker
	return t;
}
