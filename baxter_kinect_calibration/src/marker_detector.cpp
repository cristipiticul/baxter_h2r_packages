#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ar_track_alvar/MarkerDetector.h"

//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/registration/transformation_estimation_svd.h>
//#include <sensor_msgs/PointCloud2.h>

#include <boost/thread/mutex.hpp>

#include <std_srvs/Empty.h>

typedef std::vector<alvar::MarkerData, Eigen::aligned_allocator<alvar::MarkerData> > MarkerVector;

#define MARKER_DETECTION_ENABLE_SERVICE_NAME "enable_marker_detection"
#define MARKER_DETECTION_DISABLE_SERVICE_NAME "disable_marker_detection"

std::string image_topic;
std::string info_topic;
std::string camera_optical_frame;
int image_height;
int image_width;

double max_new_marker_error = 0.08; // same value as in ar_track_alvar/launch/pr2_indiv_no_kinect.launch
double max_track_error = 0.2; // same value as in ar_track_alvar/launch/pr2_indiv_no_kinect.launch
double marker_size;

tf::Transform getTransformFromPose(alvar::Pose &p);
void printStaticTransformCommand(tf::StampedTransform transform);


class SimpleDetector
{
private:
	ros::NodeHandle n;
	image_transport::ImageTransport it;

	alvar::MarkerDetector<alvar::MarkerData> detector;
	std::string cameraInfoTopic;
	std::string imageTopic;
	alvar::Camera* pCameraInfo;
	image_transport::Subscriber imageSubscriber;

	tf::TransformBroadcaster transformBroadcaster;

	ros::Rate sleepRate; //TODO: parametrize this

	ros::ServiceServer startService;
	ros::ServiceServer stopService;

	int imageHeight;
	int imageWidth;
public:
	SimpleDetector(std::string cameraInfoTopicParam, std::string imageTopicParam,
			int imageHeightParam, int imageWidthParam) :
			n(),
			it(n),
			pCameraInfo(NULL),
			cameraInfoTopic(cameraInfoTopicParam),
			imageTopic(imageTopicParam),
			imageHeight(imageHeightParam),
			imageWidth(imageWidthParam),
			sleepRate(10.0)
	{
		detector.SetMarkerSize(marker_size);
		startService = n.advertiseService(MARKER_DETECTION_ENABLE_SERVICE_NAME,
				&SimpleDetector::start, this);
		stopService = n.advertiseService(MARKER_DETECTION_DISABLE_SERVICE_NAME,
				&SimpleDetector::stop, this);
	}

	void callback(const sensor_msgs::ImageConstPtr & image_msg);
	bool start(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
	bool stop(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
	void waitForDetectionToFinish();
	MarkerVector* getDetectedMarkers();

	~SimpleDetector();
};

void usage(char* program_name)
{
	std::cout << "Usage: " << program_name << " image_topic info_topic camera_optical_frame image_height image_width marker_size\n";
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "marker_detect_rgb_only");
	if (argc != 7)
	{
		usage(argv[0]);
		return 1;
	}
	image_topic = std::string(argv[1]);
	info_topic = std::string(argv[2]);
	camera_optical_frame = std::string(argv[3]);
	image_height = atoi(argv[4]);
	image_width = atoi(argv[5]);
	if (sscanf(argv[6], "%lf", &marker_size) == 0)
	{
		std::cerr << "Could not read marker size parameter!" << '\n';
		return 1;
	}

	std::cout << "image topic: " << image_topic << '\n';
	std::cout << "info topic: " << info_topic << '\n';
	std::cout << "camera frame: " << camera_optical_frame << '\n';
	std::cout << "marker size: " << marker_size << '\n';
	std::cout << "expected image height: " << image_height << '\n';
	std::cout << "expected image width: " << image_width << '\n';

	ros::NodeHandle n;

	SimpleDetector detector(info_topic, image_topic,
			image_height, image_width);

	ros::spin();

	return 0;
}

void SimpleDetector::callback(const sensor_msgs::ImageConstPtr & image_msg)
{
	//If no camera info, return
	if (pCameraInfo == NULL || !pCameraInfo->getCamInfo_)
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
	IplImage ipl_image = cv_ptr_->image;
	detector.Detect(&ipl_image, pCameraInfo, true, true,
			max_new_marker_error, max_track_error, alvar::CVSEQ, true);

	MarkerVector* detectedMarkers = detector.markers;
	for (int i = 0; i < detector.markers->size(); i++)
	{
		std::stringstream str;
		str << "marker_" << detectedMarkers->at(i).GetId();
		tf::StampedTransform transform(getTransformFromPose(detectedMarkers->at(i).pose), ros::Time::now(), camera_optical_frame, str.str());
		transformBroadcaster.sendTransform(transform);
		std::cout << "Found marker " << str.str() << '\n';
	}

	if (ipl_image.height != imageHeight || ipl_image.width != imageWidth)
	{
		ROS_ERROR(
				"Wrist camera image is incorrect size! Should be %dx%d. Shutting down.", imageWidth, imageHeight);
		exit(1);
	}

	sleepRate.sleep();
}


bool SimpleDetector::start(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	/**
	 * TODO: Camera info will never be set if:
	 * - cameraInfo is initialized in the constructor and
	 * - the cameraInfo publisher is started after this node
	 */
	if (pCameraInfo == NULL)
	{
		pCameraInfo = new alvar::Camera(n, cameraInfoTopic);
	}
	imageSubscriber = it.subscribe(imageTopic, 1, &SimpleDetector::callback, this);
	return true;
}

bool SimpleDetector::stop(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	imageSubscriber.shutdown();
	return true;
}

SimpleDetector::~SimpleDetector()
{
	delete pCameraInfo;
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
