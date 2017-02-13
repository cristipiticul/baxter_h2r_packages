#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ar_track_alvar/MarkerDetector.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <sensor_msgs/PointCloud2.h>

typedef std::vector<alvar::MarkerData, Eigen::aligned_allocator<alvar::MarkerData> > MarkerVector;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

double max_new_marker_error = 0.08; // same value as in ar_track_alvar/launch/pr2_indiv_no_kinect.launch
double max_track_error = 0.2; // same value as in ar_track_alvar/launch/pr2_indiv_no_kinect.launch
double marker_size = 8.0;

ros::Publisher pointCloudPublisher;

tf::Transform getTransformFromPose(alvar::Pose &p);
bool getTransformFromListener(const char from[], const char to[],
		tf::StampedTransform &result);
void addMarkerToPointCloud(alvar::MarkerData marker, tf::Transform calibrationTransform, PointCloud::Ptr pointCloud);

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

	volatile bool callbackEnabled;
	volatile bool detectionDone;

	// Used to flip the image from camera (by 180 degrees).
	// In gazebo, the image is rotated by 180 degrees, so we need to correct that!
	bool flipImages;

	int imageHeight;
	int imageWidth;
public:
	SimpleDetector(std::string cameraInfoTopicParam, std::string imageTopic,
			int imageHeightParam, int imageWidthParam) :
			n(),
			it(n),
			cameraInfoTopic(cameraInfoTopicParam),
			cameraInfo(n, cameraInfoTopicParam),
			imageSubscriber(it.subscribe(imageTopic, 1, &SimpleDetector::callback, this)),
			callbackEnabled(false),
			detectionDone(false),
			imageHeight(imageHeightParam),
			imageWidth(imageWidthParam),
			flipImages(false)
	{
		detector.SetMarkerSize(marker_size);
	}

	void callback(const sensor_msgs::ImageConstPtr & image_msg);
	void startDetection();
	void waitForDetectionToFinish();
	void setFlipImages(bool value)
	{
		flipImages = value;
	}
	cv::Mat getLastImage();
	MarkerVector* getDetectedMarkers();
};

class Calibrator
{
private:
	std::string calibratedImageTopic;
	std::string calibratedInfoTopic = "/cameras/left_hand_camera/camera_info";
	std::string calibratedCameraFrame = "left_hand_camera";
	std::string calibratedCameraCalibrationFrame = "left_wrist";
	std::string uncalibratedImageTopic = "/camera/rgb/image_raw";
	std::string uncalibratedInfoTopic = "/camera/rgb/camera_info";
	std::string uncalibratedCameraFrame = "camera_rgb_optical_frame";
	std::string uncalibratedCameraCalibrationFrame = "camera_link";

	ros::NodeHandle n;


	tf::TransformListener* pTransformListener;
	tf::TransformBroadcaster* pTransformBroadcaster;

	SimpleDetector calibratedCameraDetector;
	SimpleDetector uncalibratedCameraDetector;

	PointCloud::Ptr calibratedPoints;
	PointCloud::Ptr uncalibratedPoints;

public:
	Calibrator():
		calibratedImageTopic("/cameras/left_hand_camera/image"),
		calibratedInfoTopic("/cameras/left_hand_camera/camera_info"),
		calibratedCameraFrame("left_hand_camera"),
		calibratedCameraCalibrationFrame("left_wrist"),
		uncalibratedImageTopic("/camera/rgb/image_raw"),
		uncalibratedInfoTopic("/camera/rgb/camera_info"),
		uncalibratedCameraFrame("camera_rgb_optical_frame"),
		uncalibratedCameraCalibrationFrame("camera_link"),
		n(),
		pTransformListener(new tf::TransformListener(n)),
		pTransformBroadcaster(new tf::TransformBroadcaster()),
		calibratedCameraDetector(calibratedInfoTopic, calibratedImageTopic, 800, 1280),
		uncalibratedCameraDetector(uncalibratedInfoTopic, uncalibratedImageTopic, 480, 640),
		calibratedPoints(new PointCloud()),
		uncalibratedPoints(new PointCloud())
	{

		//TODO: flip images ONLY IN GAZEBO!!!
		//^THIS is WRONG. Because the image frame != point cloud frame
		// The image should be flipped!
		// In Point cloud: x->left, y->up  , z->forward
		// In image:       x->right, y->down, z->forward
		uncalibratedCameraDetector.setFlipImages(true);
	}

	void updateCalibration()
	{
		tf::StampedTransform calibratedCameraTransform;
		tf::StampedTransform uncalibratedCameraTransform;
		bool success;
		success = getTransformFromListener(calibratedCameraCalibrationFrame.c_str(), calibratedCameraFrame.c_str(), calibratedCameraTransform);
		if (!success)
		{
			ROS_ERROR(
					"Transform calibrated_camera->robot_link was not retrieved successfully");
			return;
		}
		success = getTransformFromListener(uncalibratedCameraCalibrationFrame.c_str(), uncalibratedCameraFrame.c_str(), uncalibratedCameraTransform);
		if (!success)
		{
			ROS_ERROR(
					"Transform calibrated_camera->robot_link was not retrieved successfully");
			return;
		}
		calibratedCameraDetector.startDetection();
		uncalibratedCameraDetector.startDetection();
		calibratedCameraDetector.waitForDetectionToFinish();
		uncalibratedCameraDetector.waitForDetectionToFinish();

		MarkerVector* calibratedMarkers = calibratedCameraDetector.getDetectedMarkers();
		MarkerVector* uncalibratedMarkers = uncalibratedCameraDetector.getDetectedMarkers();
		for (int i = 0; i < calibratedMarkers->size(); i++)
		{
			for (int j = 0; j < uncalibratedMarkers->size(); j++)
			{
				if (calibratedMarkers->at(i).GetId() == uncalibratedMarkers->at(j).GetId())
				{
					addMarkerToPointCloud(calibratedMarkers->at(i), calibratedCameraTransform, calibratedPoints);
					addMarkerToPointCloud(uncalibratedMarkers->at(i), uncalibratedCameraTransform, uncalibratedPoints);
				}
			}
		}
		if (calibratedPoints->size() == 0)
		{
			ROS_INFO("No markers were detected by both cameras!");
			return;
		}
		pcl::registration::TransformationEstimationSVD<PointType, PointType, double> svd;
		pcl::registration::TransformationEstimationSVD<PointType, PointType, double>::Matrix4 resultingTransformMatrix;
		svd.estimateRigidTransformation(*uncalibratedPoints, *calibratedPoints, resultingTransformMatrix);

		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				std::cout << resultingTransformMatrix(i, j) << ' ';
			}
			std::cout << '\n';
		}

		Eigen::Affine3d resultingTransformAffine(resultingTransformMatrix);
		tf::Transform resultingTransform;
		tf::transformEigenToTF(resultingTransformAffine, resultingTransform);
		tf::Quaternion rotation(resultingTransform.getRotation());
		rotation *= tf::createQuaternionFromRPY(M_PI, 0, 0);
		resultingTransform.setRotation(rotation);
		tf::StampedTransform resultingTransformStamped(resultingTransform,
				ros::Time::now(), calibratedCameraCalibrationFrame,
				uncalibratedCameraCalibrationFrame);


		cv::imshow("calibrated_camera_image", calibratedCameraDetector.getLastImage());
		cv::imshow("uncalibrated_camera_image", uncalibratedCameraDetector.getLastImage());
		cv::waitKey(0);
	}

};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "marker_detect_rgb_only");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	pointCloudPublisher = n.advertise<sensor_msgs::PointCloud2>(
			"calibration_debug_point_cloud", 1);

	// Give transform listener time to get some data.
	ros::Duration(1.0).sleep();

	//Subscribe to topics and set up callbacks


	ros::Duration(0.5).sleep(); // Give the spinner time to get the cameras' info

/*
	bool transformIsRecent = getTransformFromListener(calibrationFrame.c_str(),
			cameraFrame.c_str(), calibrationFrameToCameraFrame);
	if (!transformIsRecent)
	{
	}
*/

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
	*/
	while (ros::ok())
	{
		resultingTransformStamped.stamp_ = ros::Time::now();
		pTransformBroadcaster->sendTransform(resultingTransformStamped);
		ros::Duration(0.1).sleep();
	}
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
		cv::Mat& image = cv_ptr_->image;
		if (flipImages)
		{
			cv::flip(image, image, -1);
		}
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
}

cv::Mat SimpleDetector::getLastImage()
{
	return lastImage;
}

MarkerVector* SimpleDetector::getDetectedMarkers()
{
	return detector.markers;
}

/*
 * For each marker, take 4 points on it (like its corners), transform them
 * according to the calibrationTranform parameter and add them to a point
 * cloud.
 */
void addMarkerToPointCloud(alvar::MarkerData marker, tf::Transform calibrationTransform, PointCloud::Ptr pointCloud)
{
	tf::Transform cameraToMarker = getTransformFromPose(marker.pose);
	tf::Transform calibrationFrameToMarker = calibrationTransform
			* cameraToMarker;

	tf::Vector3 p1(-marker_size / 400.0, -marker_size / 400.0, 0.0);
	tf::Vector3 p1InCalibrationFrame = calibrationFrameToMarker * p1;
	tf::Vector3 p2(-marker_size / 400.0, marker_size / 400.0, 0.0);
	tf::Vector3 p2InCalibrationFrame = calibrationFrameToMarker * p2;
	tf::Vector3 p3(marker_size / 400.0, marker_size / 400.0, 0.0);
	tf::Vector3 p3InCalibrationFrame = calibrationFrameToMarker * p3;
	tf::Vector3 p4(marker_size / 400.0, -marker_size / 400.0, 0.0);
	tf::Vector3 p4InCalibrationFrame = calibrationFrameToMarker * p4;

	sensor_msgs::PointCloud2 pointCloudRos;
	pointCloud->push_back(
			pcl::PointXYZ(p1InCalibrationFrame.x(), p1InCalibrationFrame.y(),
					p1InCalibrationFrame.z()));
	pointCloud->push_back(
			pcl::PointXYZ(p2InCalibrationFrame.x(), p2InCalibrationFrame.y(),
					p2InCalibrationFrame.z()));
	pointCloud->push_back(
			pcl::PointXYZ(p3InCalibrationFrame.x(), p3InCalibrationFrame.y(),
					p3InCalibrationFrame.z()));
	pointCloud->push_back(
			pcl::PointXYZ(p4InCalibrationFrame.x(), p4InCalibrationFrame.y(),
					p4InCalibrationFrame.z()));
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
