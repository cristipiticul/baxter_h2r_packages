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

#include <boost/thread/mutex.hpp>

#include <std_srvs/Empty.h>

typedef std::vector<alvar::MarkerData, Eigen::aligned_allocator<alvar::MarkerData> > MarkerVector;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

const char CALIBRATED_IMAGE_TOPIC[] = "/cameras/left_hand_camera/image";
const char CALIBRATED_INFO_TOPIC[] = "/cameras/left_hand_camera/camera_info";
const char CALIBRATED_CAMERA_FRAME[] = "left_hand_camera";
/** For wrist support
const char CALIBRATED_CAMERA_CALIBRATION_FRAME[] = "left_wrist";
*/
/** For static camera (not attached to the robot */
const char CALIBRATED_CAMERA_CALIBRATION_FRAME[] = "base";

/* For Asus Xtion
const char UNCALIBRATED_IMAGE_TOPIC[] = "/camera/rgb/image_raw";
const char UNCALIBRATED_INFO_TOPIC[] = "/camera/rgb/camera_info";
const int UNCALIBRATED_CAMERA_HEIGHT = 480;
const int UNCALIBRATED_CAMERA_WIDTH = 640;
*/
/* For Kinect2 */
const char UNCALIBRATED_IMAGE_TOPIC[] = "/camera/hd/image_color";
const char UNCALIBRATED_INFO_TOPIC[] = "/camera/hd/camera_info";
const char UNCALIBRATED_CAMERA_FRAME[] = "camera_rgb_optical_frame";
const char UNCALIBRATED_CAMERA_CALIBRATION_FRAME[] = "camera_link";
const int UNCALIBRATED_CAMERA_HEIGHT = 1080;
const int UNCALIBRATED_CAMERA_WIDTH = 1920;


double max_new_marker_error = 0.08; // same value as in ar_track_alvar/launch/pr2_indiv_no_kinect.launch
double max_track_error = 0.2; // same value as in ar_track_alvar/launch/pr2_indiv_no_kinect.launch
double marker_size = 14.4;

ros::Publisher pointCloudPublisher;

tf::Transform getTransformFromPose(alvar::Pose &p);
void addMarkerToPointCloud(alvar::MarkerData marker, tf::Transform calibrationTransform, PointCloud::Ptr pointCloud);
void printStaticTransformCommand(tf::StampedTransform transform);


class SimpleDetector
{
private:
	ros::NodeHandle n;
	image_transport::ImageTransport it;

	alvar::MarkerDetector<alvar::MarkerData> detector;
	std::string cameraInfoTopic;
	std::string imageTopic;
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
	SimpleDetector(std::string cameraInfoTopicParam, std::string imageTopicParam,
			int imageHeightParam, int imageWidthParam) :
			n(),
			it(n),
			cameraInfoTopic(cameraInfoTopicParam),
			imageTopic(imageTopicParam),
			cameraInfo(n, cameraInfoTopicParam),
			imageSubscriber(),
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
	std::string calibratedInfoTopic;
	std::string calibratedCameraFrame;
	std::string calibratedCameraCalibrationFrame;
	std::string uncalibratedImageTopic;
	std::string uncalibratedInfoTopic;
	std::string uncalibratedCameraFrame;
	std::string uncalibratedCameraCalibrationFrame;

	ros::NodeHandle n;

	tf::TransformListener* pTransformListener;

	SimpleDetector calibratedCameraDetector;
	SimpleDetector uncalibratedCameraDetector;

	PointCloud::Ptr calibratedPoints;
	PointCloud::Ptr uncalibratedPoints;

	volatile bool calibrationIsReady;

	tf::StampedTransform currentCalibration;
	boost::mutex currentCalibrationMutex;

	ros::ServiceServer updateCalibrationService;
public:
	Calibrator():
		calibratedImageTopic(CALIBRATED_IMAGE_TOPIC),
		calibratedInfoTopic(CALIBRATED_INFO_TOPIC),
		calibratedCameraFrame(CALIBRATED_CAMERA_FRAME),
		calibratedCameraCalibrationFrame(CALIBRATED_CAMERA_CALIBRATION_FRAME),
		uncalibratedImageTopic(UNCALIBRATED_IMAGE_TOPIC),
		uncalibratedInfoTopic(UNCALIBRATED_INFO_TOPIC),
		uncalibratedCameraFrame(UNCALIBRATED_CAMERA_FRAME),
		uncalibratedCameraCalibrationFrame(UNCALIBRATED_CAMERA_CALIBRATION_FRAME),
		n(),
		pTransformListener(new tf::TransformListener(n)),
		calibratedCameraDetector(calibratedInfoTopic, calibratedImageTopic, 800, 1280),
		uncalibratedCameraDetector(uncalibratedInfoTopic, uncalibratedImageTopic, UNCALIBRATED_CAMERA_HEIGHT, UNCALIBRATED_CAMERA_WIDTH),
		calibratedPoints(new PointCloud()),
		uncalibratedPoints(new PointCloud()),
		calibrationIsReady(false),
		currentCalibration(),
		currentCalibrationMutex(),
		updateCalibrationService()
	{

		//TODO: flip images ONLY IN GAZEBO!!!
		//^THIS is WRONG. Because the image frame != point cloud frame
		// The image should be flipped!
		// In Point cloud: x->left, y->up  , z->forward
		// In image:       x->right, y->down, z->forward
		//uncalibratedCameraDetector.setFlipImages(true);
	}

	void startListening()
	{
		updateCalibrationService = n.advertiseService("update_calibration", &Calibrator::updateCalibration, this);
	}

	bool updateCalibration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	bool getTransformFromListener(const char from[], const char to[],
			tf::StampedTransform &result);
	bool calibrationReady();
	tf::StampedTransform getCurrentCalibration();

	~Calibrator()
	{
		delete pTransformListener;
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "marker_detect_rgb_only");
	ros::NodeHandle n;
	tf::TransformBroadcaster* pTransformBroadcaster(new tf::TransformBroadcaster());
	ros::AsyncSpinner spinner(1);
	spinner.start();

	pointCloudPublisher = n.advertise<sensor_msgs::PointCloud2>(
			"calibration_debug_point_cloud", 1);

	Calibrator calibrator;

	ros::Duration(0.5).sleep(); // Give the spinner time to get the cameras' info

	calibrator.startListening();

	while (ros::ok())
	{
		if (calibrator.calibrationReady())
		{
			pTransformBroadcaster->sendTransform(calibrator.getCurrentCalibration());
		}
		ros::Duration(0.1).sleep();
	}
	delete pTransformBroadcaster;

	return 0;
}

void SimpleDetector::callback(const sensor_msgs::ImageConstPtr & image_msg)
{
	ROS_INFO("CALLBACK!!! %s", imageTopic.c_str());
	imageSubscriber.shutdown();
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
	imageSubscriber = it.subscribe(imageTopic, 1, &SimpleDetector::callback, this);
}

void SimpleDetector::waitForDetectionToFinish()
{
	do
	{
		ros::spinOnce(); // Without this, the topics' callbacks would not be called during the service call. IDK why..
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

bool Calibrator::updateCalibration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	tf::StampedTransform calibratedCameraTransform;
	tf::StampedTransform uncalibratedCameraTransform;
	bool success;
	success = getTransformFromListener(calibratedCameraCalibrationFrame.c_str(), calibratedCameraFrame.c_str(), calibratedCameraTransform);
	if (!success)
	{
		ROS_ERROR(
				"Transform calibrated_camera->robot_link was not retrieved successfully");
		return false;
	}
	success = getTransformFromListener(uncalibratedCameraCalibrationFrame.c_str(), uncalibratedCameraFrame.c_str(), uncalibratedCameraTransform);
	if (!success)
	{
		ROS_ERROR(
				"Transform calibrated_camera->robot_link was not retrieved successfully");
		return false;
	}
	calibratedCameraDetector.startDetection();
	uncalibratedCameraDetector.startDetection();
	ROS_INFO("Waiting for detection 1");
	calibratedCameraDetector.waitForDetectionToFinish();
	ROS_INFO("Waiting for detection 2");
	uncalibratedCameraDetector.waitForDetectionToFinish();
	ROS_INFO("Done!");

	ROS_INFO("Putting markers in pointcloud...");
	MarkerVector* calibratedMarkers = calibratedCameraDetector.getDetectedMarkers();
	MarkerVector* uncalibratedMarkers = uncalibratedCameraDetector.getDetectedMarkers();
	for (int i = 0; i < calibratedMarkers->size(); i++)
	{
		for (int j = 0; j < uncalibratedMarkers->size(); j++)
		{
			if (calibratedMarkers->at(i).GetId() == uncalibratedMarkers->at(j).GetId())
			{
				addMarkerToPointCloud(calibratedMarkers->at(i), calibratedCameraTransform, calibratedPoints);
				addMarkerToPointCloud(uncalibratedMarkers->at(j), uncalibratedCameraTransform, uncalibratedPoints);
			}
		}
	}
	ROS_INFO("Done!");
	if (calibratedPoints->size() == 0)
	{
		ROS_INFO("No markers were detected by both cameras!");
		return false;
	}
	pcl::registration::TransformationEstimationSVD<PointType, PointType, double> svd;
	pcl::registration::TransformationEstimationSVD<PointType, PointType, double>::Matrix4 resultingTransformMatrix;
	svd.estimateRigidTransformation(*uncalibratedPoints, *calibratedPoints, resultingTransformMatrix);
	ROS_INFO("SVD done!");

	Eigen::Affine3d resultingTransformAffine(resultingTransformMatrix);
	tf::Transform resultingTransform;
	tf::transformEigenToTF(resultingTransformAffine, resultingTransform);
	//tf::Quaternion rotation(resultingTransform.getRotation());
	//rotation *= tf::createQuaternionFromRPY(M_PI, 0, 0);
	//resultingTransform.setRotation(rotation);
	currentCalibrationMutex.lock();
	currentCalibration = tf::StampedTransform(resultingTransform,
			ros::Time::now(), calibratedCameraCalibrationFrame,
			uncalibratedCameraCalibrationFrame);
	currentCalibrationMutex.unlock();
	calibrationIsReady = true;

	printStaticTransformCommand(currentCalibration);

	cv::imshow("calibrated_camera_image", calibratedCameraDetector.getLastImage());
	cv::imshow("uncalibrated_camera_image", uncalibratedCameraDetector.getLastImage());
	cv::waitKey(0);

	return true;
}

bool Calibrator::calibrationReady()
{
	return calibrationIsReady;
}

bool Calibrator::getTransformFromListener(const char from[], const char to[],
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

tf::StampedTransform Calibrator::getCurrentCalibration()
{
	currentCalibrationMutex.lock();
	tf::StampedTransform result(currentCalibration);
	currentCalibrationMutex.unlock();
	result.stamp_ = ros::Time::now();
	return result;
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

void printStaticTransformCommand(tf::StampedTransform transform)
{
	std::cout << transform.frame_id_ << " -> " << transform.child_frame_id_ << '\n';
	std::cout << "rosrun tf static_transform_publisher ";
	std::cout << transform.getOrigin().x() << ' ';
	std::cout << transform.getOrigin().y() << ' ';
	std::cout << transform.getOrigin().z() << ' ';
	std::cout << transform.getRotation().x() << ' ';
	std::cout << transform.getRotation().y() << ' ';
	std::cout << transform.getRotation().z() << ' ';
	std::cout << transform.getRotation().w() << ' ';
	std::cout << transform.frame_id_ << ' ';
	std::cout << transform.child_frame_id_ << ' ';
	std::cout << "100\n";
}
