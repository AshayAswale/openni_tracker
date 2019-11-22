// openni_tracker.cpp

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <sensor_msgs/Image.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

using std::string;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";
std::string PREFIX_OPENNI = "/openni/";

bool start_code = false;

int max_window_size = 10;
int total_joints = 16;
double transform_array[16][7][100];
std::map<std::string, int> joint_numbers_map;

void initializeMap()
{
  int temp = 0;
  joint_numbers_map.insert({"head", temp++});
  joint_numbers_map.insert({"neck", temp++});
  joint_numbers_map.insert({"torso", temp++});
  
  joint_numbers_map.insert({"right_shoulder", temp++});
  joint_numbers_map.insert({"right_elbow", temp++});
  joint_numbers_map.insert({"right_hand", temp++});
  
  joint_numbers_map.insert({"left_shoulder", temp++});
  joint_numbers_map.insert({"left_elbow", temp++});
  joint_numbers_map.insert({"left_hand", temp++});
  
  joint_numbers_map.insert({"right_hip", temp++});
  joint_numbers_map.insert({"right_knee", temp++});
  joint_numbers_map.insert({"right_foot", temp++});
  
  joint_numbers_map.insert({"left_hip", temp++});
  joint_numbers_map.insert({"left_knee", temp++});
  joint_numbers_map.insert({"left_foot", temp++});
  joint_numbers_map.insert({"pelvis", temp++});
}

double getAvgValue(const int joint_number, const int coord)
{
  static int curr_size = 0;
  if(joint_number == 0 && curr_size<max_window_size)
    curr_size++;
  
  double value = 0;
  for (int read = 0; read < curr_size;read++)
  {
    value += transform_array[joint_number][coord][read];
  }
  
  return value / max_window_size;
}

void movingAvgFilter(const std::string& frame_name, tf::Transform& transform)
{
  int joint_number = joint_numbers_map[frame_name];
  
  static int i = 0;

  transform_array[joint_number][0][i] = transform.getOrigin().getX();
  transform_array[joint_number][1][i] = transform.getOrigin().getY();
  transform_array[joint_number][2][i] = transform.getOrigin().getZ();
  
  transform_array[joint_number][3][i] = transform.getRotation().getX();
  transform_array[joint_number][4][i] = transform.getRotation().getY();
  transform_array[joint_number][5][i] = transform.getRotation().getZ();
  transform_array[joint_number][6][i] = transform.getRotation().getW();

  // Completion of one cycle
  if(joint_number == 0)
  {
    // Restart the window for average
    i = i < max_window_size - 1 ? i+1 : 0;
  }    

  tf::Vector3 vector;
  vector.setX(getAvgValue(joint_number, 0));
  vector.setY(getAvgValue(joint_number, 1));
  vector.setZ(getAvgValue(joint_number, 2));

  tf::Quaternion quat;
  quat.setX(getAvgValue(joint_number, 3));
  quat.setY(getAvgValue(joint_number, 4));
  quat.setZ(getAvgValue(joint_number, 5));
  quat.setW(getAvgValue(joint_number, 6));

  transform.setOrigin(vector);
  transform.setRotation(quat);
}

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) {
    static tf::TransformBroadcaster br;
    static tf::Transform left_hip_final_transform, right_hip_final_transform, pelvis_final_transform;
    static tf::Transform left_shoulder_transform, right_shoulder_transform, neck_transform;
    static tf::Transform left_elbow_transform, right_elbow_transform;
    static tf::Vector3 pelvis_vector, temp_vector;

    if (child_frame_id.compare("pelvis")==0)
    {
    pelvis_vector = left_hip_final_transform.getOrigin() / 2 + right_hip_final_transform.getOrigin() / 2;
    pelvis_final_transform.setRotation(tf::Quaternion(0.0, 0.0, M_PI));
    pelvis_final_transform.setOrigin(pelvis_vector);
    br.sendTransform(
        tf::StampedTransform(pelvis_final_transform, ros::Time::now(), frame_id, PREFIX_OPENNI+child_frame_id));
    return;
    }

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    // XnSkeletonJointOrientation joint_orientation;
    // g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    // XnFloat* m = joint_orientation.orientation.elements;
    // KDL::Rotation rotation(m[0], m[1], m[2],
    // 					   m[3], m[4], m[5],
    // 					   m[6], m[7], m[8]);
    // double qx, qy, qz, qw;
    // rotation.GetQuaternion(qx, qy, qz, qw);

    // char child_frame_no[128];
    // snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);
    //##########################
    // CHANGE HERE IF REQUIRED!!

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    // tf::Quaternion(pitch, roll, yaw);
    transform.setRotation(tf::Quaternion(M_PI/2, -M_PI/2, 0.0));
    // transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(M_PI/2, 0, M_PI/2);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    if (child_frame_id.compare("neck") == 0)
      neck_transform = transform;
    if (child_frame_id.compare("left_hip") == 0)
      left_hip_final_transform = transform;
    else if (child_frame_id.compare("right_hip") == 0)
      right_hip_final_transform = transform;
    else if (child_frame_id.compare("left_shoulder") == 0)
    {
      left_shoulder_transform = transform;
      temp_vector = neck_transform.getOrigin() - pelvis_final_transform.getOrigin();
      tfScalar roll = atan2(temp_vector.getY(), temp_vector.getZ());
      transform.setRotation(tf::Quaternion(0.0, -roll, M_PI));
    }
    else if (child_frame_id.compare("right_shoulder") == 0)
    {
      right_shoulder_transform = transform;
      temp_vector = neck_transform.getOrigin() - pelvis_final_transform.getOrigin();
      tfScalar roll = atan2(temp_vector.getY(), temp_vector.getZ());
      transform.setRotation(tf::Quaternion(0.0, -roll, M_PI));
    }
    else if (child_frame_id.compare("left_elbow") == 0)
    {
      temp_vector = transform.getOrigin() - left_shoulder_transform.getOrigin();
      tfScalar yaw = atan2(temp_vector.getX(), temp_vector.getY());
      tfScalar roll = atan2(temp_vector.getY(), temp_vector.getZ());
      //   transform.setRotation(tf::Quaternion(0.0, M_PI/2 - roll, -M_PI/2 + yaw));
      transform.setRotation(tf::Quaternion(0.0, -M_PI/2-roll, -yaw));
      left_elbow_transform = transform;
    }
    else if (child_frame_id.compare("right_elbow") == 0)
    {
      temp_vector = transform.getOrigin() - right_shoulder_transform.getOrigin();
      tfScalar yaw = atan2(temp_vector.getX(), temp_vector.getY());
      tfScalar roll = atan2(temp_vector.getZ(), temp_vector.getY());
      //   transform.setRotation(tf::Quaternion(0.0, M_PI/2 - roll, -M_PI/2 + yaw));
      transform.setRotation(tf::Quaternion(0.0, roll, M_PI-yaw));
      right_elbow_transform = transform;
    }
    else if (child_frame_id.compare("left_hand") == 0)
    {
      temp_vector = transform.getOrigin() - left_elbow_transform.getOrigin();
      tfScalar yaw = atan2(temp_vector.getX(), temp_vector.getY());
      tfScalar roll = atan2(temp_vector.getY(), temp_vector.getZ());
      tfScalar pitch = atan2(temp_vector.getX(), temp_vector.getZ());
      transform.setRotation(tf::Quaternion(0.0, -M_PI/2-roll, -yaw));
      // transform.setRotation(tf::Quaternion(0.0, M_PI/2 - roll, -M_PI/2 + yaw));
      // tfScaler pitch = atan2(temp_vector.getX(), temp_vector.getZ());
      // transform.setRotation(tf::Quaternion(-pitch, -M_PI/2-roll, -yaw));
      // ROS_INFO_STREAM("Pitch: " << pitch);
    }
    else if (child_frame_id.compare("right_hand") == 0)
    {
      temp_vector = transform.getOrigin() - right_elbow_transform.getOrigin();
      tfScalar yaw = atan2(temp_vector.getX(), temp_vector.getY());
      tfScalar roll = atan2(temp_vector.getZ(), temp_vector.getY());
      //   transform.setRotation(tf::Quaternion(0.0, M_PI/2 - roll, -M_PI/2 + yaw));
      transform.setRotation(tf::Quaternion(0.0, roll, M_PI-yaw));
    }
    // else if (child_frame_id.compare("torso") == 0)
    // {
    //   temp_vector = transform.getOrigin() - pelvis_final_transform.getOrigin();
    //   tfScalar yaw = atan2(temp_vector.getX(), temp_vector.getY());
    //   tfScalar roll = atan2(temp_vector.getZ(), temp_vector.getY());
    //   tfScalar roll = atan2(temp_vector.getZ(), temp_vector.getY());
    //   //   transform.setRotation(tf::Quaternion(0.0, M_PI/2 - roll, -M_PI/2 + yaw));
    //   transform.setRotation(tf::Quaternion(0.0, roll, M_PI-yaw));
    // }

    movingAvgFilter(child_frame_id, transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, PREFIX_OPENNI+child_frame_id));
}

void publishTransforms(const std::string& frame_id) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;
        publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
        publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");

        publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "right_shoulder");
        publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "right_elbow");
        publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "right_hand");

        publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "left_shoulder");
        publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "left_elbow");
        publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "left_hand");

        publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "right_hip");
        publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "right_knee");
        publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "right_foot");

        publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "left_hip");
        publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "left_knee");
        publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "left_foot");
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "pelvis");
    }
}

void rgb_image_cb(sensor_msgs::Image image)
{
  start_code = true;
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

int main(int argc, char **argv) {
    ros::init(argc, argv, "openni_tracker");
    ros::NodeHandle nh;

    ros::Subscriber subscribe = nh.subscribe("/camera/rgb/image_color", 1, rgb_image_cb);
    ros::Rate loop_rate(10.0);

    while (ros::ok() && !start_code)
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    string configFilename = ros::package::getPath("openni_tracker") + "/openni_tracker.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
	    if (nRetVal != XN_STATUS_OK) {
		    ROS_ERROR("NITE is likely missing: Please install NITE >= 1.5.2.21. Check the readme for download information. Error Info: User generator failed: %s", xnGetStatusString(nRetVal));
            return nRetVal;
	    }
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		ROS_INFO("Supplied user generator doesn't support skeleton");
		return 1;
	}

    XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	ros::Rate r(100);
  initializeMap();

  ros::NodeHandle pnh("~");
  string frame_id("openni_depth_frame");
  pnh.getParam("camera_frame_id", frame_id);
                
	while (ros::ok()) {
		g_Context.WaitAndUpdateAll();
		publishTransforms(frame_id);
		r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}
