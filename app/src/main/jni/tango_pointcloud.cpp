/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define GLM_FORCE_RADIANS

#include <jni.h>
#include <string>

#include "tango-gl/axis.h"
#include "tango-gl/camera.h"
#include "tango-gl/color.h"
#include "tango-gl/frustum.h"
#include "tango-gl/grid.h"
#include "tango-gl/transform.h"
#include "tango-gl/util.h"

#include "pointcloud.h"
#include "tango_data.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <sys/types.h>
#include <stdint.h>
#include <fcntl.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// Screen size.
GLuint screen_width;
GLuint screen_height;

// Render camera's parent transformation.
// This object is a pivot transformtion for render camera to rotate around.
tango_gl::Transform* cam_parent_transform;

// Render camera.
tango_gl::Camera* cam;

// Device frustum.
tango_gl::Frustum* frustum;

// Point cloud drawable object.
Pointcloud* pointcloud;

// Device axis (in OpenGL coordinates).
tango_gl::Axis* axis;

// Ground grid.
// Each block is 1 meter by 1 meter in real world.
tango_gl::Grid* grid;

// Single finger touch positional values.
// First element in the array is x-axis touching position.
// Second element in the array is y-axis touching position.
float cam_start_angle[2];
float cam_cur_angle[2];

// Double finger touch distance value.
float cam_start_dist;
float cam_cur_dist;

enum CameraType {
  FIRST_PERSON = 0,
  THIRD_PERSON = 1,
  TOP_DOWN = 2
};
CameraType camera_type;

// Render and camera controlling constant values.
// Height offset is used for offset height of motion tracking
// pose data. Motion tracking start position is (0,0,0). Adding
// a height offset will give a more reasonable pose while a common
// human is holding the device. The units is in meters.
const glm::vec3 kHeightOffset = glm::vec3(0.0f, 1.3f, 0.0f);

// Render camera observation distance in third person camera mode.
const float kThirdPersonCameraDist = 7.0f;

// Render camera observation distance in top down camera mode.
const float kTopDownCameraDist = 5.0f;

// Zoom in speed.
const float kZoomSpeed = 10.0f;

// Min/max clamp value of camera observation distance.
const float kCamViewMinDist = 1.0f;
const float kCamViewMaxDist = 100.f;

// FOV set up values.
// Third and top down camera's FOV is 65 degrees.
// First person camera's FOV is 37.8 degrees.
// The first person FOV is set to similar FOV of physical camera.
const float kHighFov = 65.0f;
const float kLowFov = 37.8f;

// Frustum scale.
const glm::vec3 kFrustumScale = glm::vec3(0.4f, 0.3f, 0.5f);

// Color of the ground grid.
const tango_gl::Color kGridColor(0.85f, 0.85f, 0.85f);

enum AppMessageTypes {
  Unknown,
  TangoIntro,
  TangoPose,
  TangoPoints
};

class AppMessage {
  public:
    uint8_t type;
    uint32_t length;
    char *data;
    char *payload;

    AppMessage() {
      this->data = this->payload = NULL;
    }
    ~AppMessage() {
      if (this->data) {
        delete[] this->data;
      }
    }
    uint32_t packetLength() {
      return this->headerLength() + this->payloadLength();
    }
    uint32_t headerLength() {
      int padding = sizeof(uint8_t) * 3;
      return sizeof(uint8_t) + sizeof(uint32_t) + padding;
    }
    virtual uint32_t payloadLength() {
      return 0;
    }
    char *encodePacket() {
      uint32_t payloadlen = this->payloadLength();
      uint32_t headerlen = this->headerLength();
      uint32_t packetlen = this->packetLength();

      this->data = new char[packetlen];
      //LOGI("allocated %d bytes (%p - %p)", packetlen, this->data, this->data + packetlen);
      memcpy(this->data, &this->type, sizeof(uint8_t));
      memcpy(this->data+1, &payloadlen, sizeof(uint32_t));

      this->payload = this->data + headerlen;
      memset(this->payload, 0, payloadlen);
      //LOGI("set payload pointer (%p)", this->payload);

      this->encodePayload();

      return this->data;
    }
    virtual void encodePayload() {
    }
};

// Socket stuff
class AppServer {
  public:
    bool connectTo(const char *serverHost, int serverPort=9917) {
      this->sockfd = socket(AF_INET, SOCK_STREAM, 0);
      fcntl(this->sockfd, F_SETFL, O_NONBLOCK);

      struct sockaddr_in address;
      address.sin_family = AF_INET;
      /* save the server IP (input from Java */
      struct hostent *server = gethostbyname(serverHost);
      bcopy((char *)server->h_addr, (char *)&address.sin_addr.s_addr, server->h_length);

      /* set port */
      address.sin_port = htons(serverPort);
      memset(address.sin_zero, 0, 8);

      connect(sockfd, (struct sockaddr *) &address, sizeof(address));
    }
    bool sendMessage(AppMessage *msg) {
      send(this->sockfd, msg->encodePacket(), msg->packetLength(), 0);
    }
  protected:
    int sockfd;

};

class AppMessageTangoIntro : public AppMessage {
  public:
    const char *id;
    AppMessageTangoIntro(const char *id) : AppMessage() {
      this->id = id;
      this->type = AppMessageTypes::TangoIntro;
    }
    virtual uint32_t payloadLength() {
      return strlen(this->id) + sizeof(float) * 2 + sizeof(double);
    }
    virtual void encodePayload() {
      size_t len = strlen(this->id);

      TangoCameraIntrinsics intrinsics;
      TangoService_getCameraIntrinsics(TANGO_CAMERA_DEPTH, &intrinsics);

      float fov = 2 * atan2(0.5 * intrinsics.height, intrinsics.fy) * 180/M_PI;
      float aspect = intrinsics.width / intrinsics.height;

      memcpy(this->payload, &fov, sizeof(float));
      memcpy(this->payload + sizeof(float), &aspect, sizeof(float));
      memcpy(this->payload + 2 * sizeof(float), &intrinsics.fy, sizeof(double));
      memcpy(this->payload + 2 * sizeof(float) + sizeof(double), this->id, len);
    }
};
class AppMessageTangoPose : public AppMessage {
  public:
    double translation[3];
    double orientation[4];

    AppMessageTangoPose(double *translation, double *orientation) : AppMessage() {
      this->type = AppMessageTypes::TangoPose;

      this->translation[0] = translation[0];
      this->translation[1] = translation[1];
      this->translation[2] = translation[2];

      this->orientation[0] = orientation[0];
      this->orientation[1] = orientation[1];
      this->orientation[2] = orientation[2];
      this->orientation[3] = orientation[3];
    }
    virtual uint32_t payloadLength() {
      return sizeof(double) * 7;
    }
    virtual void encodePayload() {
      memcpy(this->payload, this->translation, sizeof(double) * 3);
      memcpy(this->payload + sizeof(double) * 3, this->orientation, sizeof(double) * 4);
    }
};
class AppMessageTangoPoints : public AppMessage {
  public:
    uint32_t totalpoints;
    uint32_t maxpoints;
    uint32_t numpoints;
    float *pointdata;
    TangoImageBuffer imagebuf;
    //pcl::PCLPointCloud2::Ptr cloud;
    sensor_msgs::PointCloud2::Ptr cloud;
    sensor_msgs::PointCloud2::Ptr cloud_filtered;

    AppMessageTangoPoints(uint32_t totalpoints, float *pointdata) : AppMessage() {
      this->type = AppMessageTypes::TangoPoints;

      this->totalpoints = totalpoints;
      this->maxpoints = 1250;
      this->numpoints = (totalpoints < this->maxpoints ? totalpoints : this->maxpoints);
      this->pointdata = pointdata;

      sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2());
      sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2());
      this->cloud = cloud;
      this->cloud_filtered = cloud_filtered;

      cloud->width = this->totalpoints;
      cloud->height = 1;

      cloud->fields.resize(3);

      // Set x/y/z as the only fields
      cloud->fields[0].name = "x";
      cloud->fields[1].name = "y";
      cloud->fields[2].name = "z";

      int offset = 0;
      // All offsets are *4, as all field data types are float32
      for (size_t d = 0; d < cloud->fields.size(); ++d, offset += 4)
      {
        cloud->fields[d].count    = 1;
        cloud->fields[d].offset   = offset;
        cloud->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      }

      cloud->point_step = offset;
      cloud->row_step   = cloud->point_step * cloud->width;

      cloud->data.resize(this->totalpoints * cloud->point_step);
      cloud->is_bigendian = false;
      cloud->is_dense     = true;

/*
      for (int i = 0; i < this->totalpoints; i++) {
        pcl::PointXYZ point(pointdata[i * 3], pointdata[i * 3 + 1], pointdata[i * 3 + 2]);
        //cloud->points.push_back(point);
      }
*/
      memcpy(&cloud->data[0], this->pointdata, cloud->point_step * this->totalpoints);

      pcl::VoxelGrid< sensor_msgs::PointCloud2 > sor;
      sor.setInputCloud (cloud);
      sor.setLeafSize (0.05f, 0.05f, 0.05f);

      sor.filter (*this->cloud_filtered);

      this->numpoints = this->cloud_filtered->width;

      //this->imagebuf = imagebuf;
    }
    virtual uint32_t payloadLength() {
      return sizeof(uint32_t) + this->numpoints * ((sizeof(float) + sizeof(uint8_t)) * 3);
    }
    virtual void encodePayload() {
      memcpy(this->payload, &this->numpoints, sizeof(uint32_t));

      float *sampledpoints = (float *) (this->payload + sizeof(uint32_t));
/*
      // Random sampling
      memcpy(sampledpoints, this->pointdata, this->numpoints * sizeof(float) * 3);
      if (this->numpoints < this->totalpoints) {
        srand(time(NULL));
        for (int i = this->numpoints; i < this->totalpoints; i++) {
          int j = rand() % (i+1);
          if (j < this->numpoints) {
            sampledpoints[j * 3    ] = this->pointdata[i * 3    ];
            sampledpoints[j * 3 + 1] = this->pointdata[i * 3 + 1];
            sampledpoints[j * 3 + 2] = this->pointdata[i * 3 + 2];
          }
        }
      }
*/
      memcpy(sampledpoints, &this->cloud_filtered->data[0], this->numpoints * sizeof(float) * 3);
      //LOGI("burp %f", sampledpoints[0] );
/*
      LOGI("derp %p", sampledpoints);
      //sampledpoints[0] = 123.45;
    delete[] sampledpoints;
*/
      uint8_t *rgb = (uint8_t *) (this->payload + (sizeof(uint32_t) + sizeof(float) * this->numpoints * 3));
      //memset(rgb, 0, sizeof(uint32_t) * this->numpoints);
      
      pthread_mutex_lock(&TangoData::GetInstance().frame_mutex);
      TangoImageBuffer *imgbuf = &TangoData::GetInstance().full_frame_data;
      uint8_t *framedataYUV = TangoData::GetInstance().cur_frame_data;
      uint8_t framedata[imgbuf->width * imgbuf->height * 3];

uint8_t* pData = TangoData::GetInstance().cur_frame_data;
uint8_t* iData = framedata;
int size = (int)(imgbuf->stride * imgbuf->height);
float invByte = 0.0039215686274509803921568627451;  // ( 1 / 255)

int halfi, uvOffset, halfj, uvOffsetHalfj;
float y_scaled, v_scaled, u_scaled;
int uOffset = size / 4 + size;
int halfstride = imgbuf->stride / 2;
for (int i = 0; i < imgbuf->height; ++i)
{
    halfi = i / 2;
    uvOffset = halfi * halfstride;
    for (int j = 0; j < imgbuf->width; ++j)
    {
        halfj = j / 2;
        uvOffsetHalfj = uvOffset + halfj;
        y_scaled = pData[i * imgbuf->stride + j] * invByte;
/*
        v_scaled = 2 * (pData[uvOffsetHalfj + size] * invByte - 0.5f) * 1;
        u_scaled = 2 * (pData[uvOffsetHalfj + uOffset] * invByte - 0.5f) * 1;
        *iData++ = (uint8_t)((y_scaled + 1.13983f * v_scaled) * 255.0);;
        *iData++ = (uint8_t)((y_scaled - 0.39465f * u_scaled - 0.58060f * v_scaled) * 255.0);
        *iData++ = (uint8_t)((y_scaled + 2.03211f * u_scaled) * 255.0);
*/
        //*iData++ = 255;
    }
}


if (false) {
      for (int i = 0; i < this->numpoints; i++) {
        int xyzidx = i * 3;
        int rgbidx = i * 3;

        float *xyz = sampledpoints + xyzidx;
        int32_t cx = (int32_t) ((xyz[0] + 1) * 0.5 * imgbuf->width);
        int32_t cy = (int32_t) ((xyz[1] + 1) * 0.5 * imgbuf->height);
        
        if (cx >= 0 && cx <= imgbuf->width && cy >= 0 && cy <= imgbuf->height) {
          //uint8_t *foo = imgbuf->data + (imgbuf->width * cy + cx);
          uint8_t *pixel = framedata + ((cy * imgbuf->stride + cx) * 3);
          //uint8_t pixel[3];
          LOGI("hello yeah %d (%f, %f) => (%d, %d) = [%x, %x, %x]", i, xyz[0], xyz[1], cx, cy, pixel[0], pixel[1], pixel[2]);
          //uint8_t color[3]
          //memcpy(rgb + rgbidx, pixel, sizeof(uint8_t) * 3);
/*
          rgb[rgbidx] = pixel[0];
          rgb[rgbidx+1] = pixel[1];
          rgb[rgbidx+2] = pixel[2];
*/

        }
      }
}
      pthread_mutex_unlock(&TangoData::GetInstance().frame_mutex);
    }
};

class AppCloudFilter {
public:
  sensor_msgs::PointCloud2 output;
  virtual sensor_msgs::PointCloud2::Ptr filter(sensor_msgs::PointCloud2::Ptr input);
};
class AppCloudFilterRandomSample : public AppCloudFilter {
public:
  virtual sensor_msgs::PointCloud2::Ptr filter(sensor_msgs::PointCloud2::Ptr input) {
    this->output.width = input.width;
    this->output.height = input.height;
    this->output.data.resize(input.width * input.height);

    // Random sampling
    memcpy(this->output.data[0], this->pointdata, this->numpoints * sizeof(float) * 3);
    if (this->numpoints < this->totalpoints) {
      srand(time(NULL));
      for (int i = this->numpoints; i < this->totalpoints; i++) {
        int j = rand() % (i+1);
        if (j < this->numpoints) {
          sampledpoints[j * 3    ] = this->pointdata[i * 3    ];
          sampledpoints[j * 3 + 1] = this->pointdata[i * 3 + 1];
          sampledpoints[j * 3 + 2] = this->pointdata[i * 3 + 2];
        }
      }
    }
  }
};
class AppCloudFilterCleanup : public AppCloudFilter {
};
class AppCloudFilterVoxelize : public AppCloudFilter {
};

AppServer appserver;

// Set camera type, set render camera's parent position and rotation.
void SetCamera(CameraType camera_index) {
  camera_type = camera_index;
  cam_cur_angle[0] = cam_cur_angle[1] = cam_cur_dist = 0.0f;
  switch (camera_index) {
    case CameraType::FIRST_PERSON:
      cam->SetFieldOfView(kLowFov);
      cam_parent_transform->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
      cam_parent_transform->SetRotation(glm::quat(1.0f, 0.0f, 0.0, 0.0f));
      break;
    case CameraType::THIRD_PERSON:
      cam->SetFieldOfView(kHighFov);
      cam->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
      cam->SetRotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
      cam_cur_dist = kThirdPersonCameraDist;
      cam_cur_angle[0] = -M_PI / 4.0f;
      cam_cur_angle[1] = M_PI / 4.0f;
      break;
    case CameraType::TOP_DOWN:
      cam->SetFieldOfView(kHighFov);
      cam->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
      cam->SetRotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
      cam_cur_dist = kTopDownCameraDist;
      cam_cur_angle[1] = M_PI / 2.0f;
      break;
    default:
      break;
  }
}

bool InitGlContent() {
  camera_type = CameraType::FIRST_PERSON;

  cam = new tango_gl::Camera();
  pointcloud = new Pointcloud();
  frustum = new tango_gl::Frustum();
  axis = new tango_gl::Axis();
  grid = new tango_gl::Grid();
  cam_parent_transform = new tango_gl::Transform();

  frustum->SetScale(kFrustumScale);

  // Set the parent-child camera transfromation.
  cam->SetParent(cam_parent_transform);

  // Put the grid at the resonable height since the motion
  // tracking pose always starts at (0, 0, 0).
  grid->SetPosition(kHeightOffset);
  grid->SetColor(kGridColor);
  SetCamera(CameraType::FIRST_PERSON);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  return true;
}

void onXYZijAvailable(void *foo, const TangoXYZij* data) {
}

bool SetupGraphics(int w, int h) {
  screen_width = w;
  screen_height = h;

  if (h == 0) {
    LOGE("Setup graphic height not valid");
    return false;
  }
  cam->SetAspectRatio(static_cast<float>(w) / static_cast<float>(h));

  //appserver.connectTo("192.168.42.187"); 
  appserver.connectTo("meobets.com"); 

  AppMessageTangoIntro intromsg("hellotango");
  appserver.sendMessage(&intromsg);

  //TangoService_connectOnXYZijAvailable(onXYZijAvailable);

  return true;
}

void UpdateNetworkPointcloud() {
    AppMessageTangoPoints msg(TangoData::GetInstance().depth_buffer_size, TangoData::GetInstance().depth_buffer);
    appserver.sendMessage((AppMessage *) &msg);
}
void UpdateNetworkPose(double ts=0) {
    TangoPoseData *pose;
    if (ts == 0) {
        pose = &TangoData::GetInstance().cur_pose_data;
    } else {
        TangoPoseData tpose;
        TangoCoordinateFramePair pairs;
        pairs.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
        pairs.target = TANGO_COORDINATE_FRAME_DEVICE;
        if (TangoService_getPoseAtTime(ts, pairs, &tpose) !=
            TANGO_SUCCESS) {
          LOGE("TangoService_getPoseAtTime(): Failed");
        }
        pose = &tpose;
    }

    AppMessageTangoPose msg(pose->translation, pose->orientation);
    appserver.sendMessage((AppMessage *) &msg);
}

// GL render loop.
bool RenderFrame() {
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  // XYZij dirty indicates that the XYZij data has been changed.
  if (TangoData::GetInstance().is_xyzij_dirty) {
    TangoData::GetInstance().UpdateXYZijData();

    UpdateNetworkPose(TangoData::GetInstance().prev_depth_timestamp);
    UpdateNetworkPointcloud();
  }

  // Pose dirty indicates that the pose data has been changed.
  if (TangoData::GetInstance().is_pose_dirty) {
    TangoData::GetInstance().UpdatePoseData();
    //UpdateNetworkPose();
  }

  /// Viewport set to full screen, and camera at origin
  /// facing on negative z direction, y axis is the up
  /// vector of the camera.
  glViewport(0, 0, screen_width, screen_height);

  // Get OpenGL camera to OpenGL world transformation for motion tracking.
  // Computed based on pose callback.
  glm::mat4 oc_2_ow_mat_motion = glm::mat4(1.0f);

  // Get OpenGL camera to OpenGL world transformation for depth.
  // Note that this transformation is different from the oc_2_ow_mat_motion
  // due to the timestamp differences. This transformation is computed
  // based on the closest pose data of depth frame timestamp.
  glm::mat4 oc_2_ow_mat_depth = glm::mat4(1.0f);

  if (camera_type == CameraType::FIRST_PERSON) {
    // Get motion transformation.
    oc_2_ow_mat_motion = TangoData::GetInstance().GetOC2OWMat(false);

    // Set camera's pose to motion tracking's pose.
    cam->SetTransformationMatrix(oc_2_ow_mat_motion);

    // Get depth frame transformation.
    oc_2_ow_mat_depth = TangoData::GetInstance().GetOC2OWMat(true);
  } else {
    // Get parent camera's rotation from touch.
    // Note that the render camera is a child transformation
    // of the this transformation.
    // cam_cur_angle[0] is the x-axis touch, cooresponding to y-axis rotation.
    // cam_cur_angle[0] is the y-axis touch, cooresponding to x-axis rotation.
    glm::quat parent_cam_rot =
        glm::rotate(glm::quat(1.0f, 0.0f, 0.0f, 0.0f), -cam_cur_angle[0],
                    glm::vec3(0, 1, 0));
    parent_cam_rot =
        glm::rotate(parent_cam_rot, -cam_cur_angle[1], glm::vec3(1, 0, 0));

    // Get motion transformation.
    oc_2_ow_mat_motion = TangoData::GetInstance().GetOC2OWMat(false);

    // Get depth frame transformation.
    oc_2_ow_mat_depth = TangoData::GetInstance().GetOC2OWMat(true);

    // Set render camera parent position and rotation.
    cam_parent_transform->SetRotation(parent_cam_rot);
    cam_parent_transform->SetPosition(
        tango_gl::util::GetTranslationFromMatrix(oc_2_ow_mat_motion));

    frustum->SetTransformationMatrix(oc_2_ow_mat_motion);
    frustum->SetScale(kFrustumScale);
    frustum->Render(cam->GetProjectionMatrix(), cam->GetViewMatrix());

    // Set camera view distance, based on touch interaction.
    cam->SetPosition(glm::vec3(0.0f, 0.0f, cam_cur_dist));
  }

  // Set axis transformation, axis representing device's pose.
  axis->SetTransformationMatrix(oc_2_ow_mat_motion);
  axis->Render(cam->GetProjectionMatrix(), cam->GetViewMatrix());

  // Render point cloud based on depth buffer and depth frame transformation.
  pointcloud->Render(
      cam->GetProjectionMatrix(), cam->GetViewMatrix(), oc_2_ow_mat_depth,
      TangoData::GetInstance().depth_buffer_size * 3,
      static_cast<float*>(TangoData::GetInstance().depth_buffer));


  grid->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f) - kHeightOffset);
  // Render grid.
  grid->Render(cam->GetProjectionMatrix(), cam->GetViewMatrix());
  return true;
}


#ifdef __cplusplus
extern "C" {
#endif
// Tango Service interfaces.
JNIEXPORT jint JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_initialize(
    JNIEnv* env, jobject, jobject activity) {
  TangoErrorType err = TangoData::GetInstance().Initialize(env, activity);
  if (err != TANGO_SUCCESS) {
    if (err == TANGO_INVALID) {
      LOGE("Tango Service version mis-match");
    } else {
      LOGE("Tango Service initialize internal error");
    }
  }
  return static_cast<int>(err);
}

JNIEXPORT void JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_setupConfig(
    JNIEnv*, jobject) {
  if (!TangoData::GetInstance().SetConfig()) {
    LOGE("Tango set config failed");
  }
}

JNIEXPORT void JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_connectCallbacks(
    JNIEnv*, jobject) {
  if (!TangoData::GetInstance().ConnectCallbacks()) {
    LOGE("Tango ConnectCallbacks failed");
  }
}

JNIEXPORT jint JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_connect(
    JNIEnv*, jobject) {
  TangoErrorType err = TangoData::GetInstance().Connect();
  if (err != TANGO_SUCCESS) {
    LOGE("Tango Service connect failed");
  }
  return static_cast<int>(err);
}

JNIEXPORT void JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_setupExtrinsics(
    JNIEnv*, jobject) {
  // The extrinsics can only be queried after connected to service.
  if (!TangoData::GetInstance().SetupExtrinsicsMatrices()) {
    LOGE("Tango set extrinsics failed");
  }
}

JNIEXPORT void JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_disconnect(
    JNIEnv*, jobject) {
  TangoData::GetInstance().Disconnect();
}

JNIEXPORT void JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_freeGLContent(
    JNIEnv*, jobject) {
  if (cam != NULL) {
    delete cam;
  }
  cam = NULL;

  if (pointcloud != NULL) {
    delete pointcloud;
  }
  pointcloud = NULL;

  if (axis != NULL) {
    delete axis;
  }
  axis = NULL;

  if (grid != NULL) {
    delete grid;
  }
  grid = NULL;

  if (frustum != NULL) {
    delete frustum;
  }
  frustum = NULL;

  if (cam_parent_transform != NULL) {
    delete cam_parent_transform;
  }
  cam_parent_transform = NULL;
}

// Graphic interfaces.
JNIEXPORT void JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_initGlContent(
    JNIEnv*, jobject) {
  InitGlContent();
}

JNIEXPORT void JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_setupGraphic(
    JNIEnv*, jobject, jint width, jint height) {
  SetupGraphics(width, height);
}

JNIEXPORT void JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_render(
    JNIEnv*, jobject) {
  RenderFrame();
}

JNIEXPORT void JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_setCamera(
    JNIEnv*, jobject, int camera_index) {
  SetCamera(static_cast<CameraType>(camera_index));
}

// Tango data interfaces.
JNIEXPORT jstring JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_getPoseString(
    JNIEnv* env, jobject) {
  pthread_mutex_lock(&TangoData::GetInstance().pose_mutex);
  std::string ret_string = TangoData::GetInstance().pose_string;
  pthread_mutex_unlock(&TangoData::GetInstance().pose_mutex);
  return (env)->NewStringUTF(ret_string.c_str());
}

JNIEXPORT jstring JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_getEventString(
    JNIEnv* env, jobject) {
  pthread_mutex_lock(&TangoData::GetInstance().event_mutex);
  std::string ret_string = TangoData::GetInstance().event_string;
  pthread_mutex_unlock(&TangoData::GetInstance().event_mutex);
  return (env)->NewStringUTF(ret_string.c_str());
}

JNIEXPORT jstring JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_getVersionNumber(
    JNIEnv* env, jobject) {
  return (env)
      ->NewStringUTF(TangoData::GetInstance().lib_version_string.c_str());
}

JNIEXPORT jint JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_getVerticesCount(
    JNIEnv*, jobject) {
  pthread_mutex_lock(&TangoData::GetInstance().xyzij_mutex);
  int ret_val = TangoData::GetInstance().depth_buffer_size;
  pthread_mutex_unlock(&TangoData::GetInstance().xyzij_mutex);
  return ret_val;
}

JNIEXPORT float JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_getAverageZ(
    JNIEnv*, jobject) {
  pthread_mutex_lock(&TangoData::GetInstance().xyzij_mutex);
  float ret_val = TangoData::GetInstance().depth_average_length;
  pthread_mutex_unlock(&TangoData::GetInstance().xyzij_mutex);
  return ret_val;
}

JNIEXPORT float JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_getFrameDeltaTime(
    JNIEnv*, jobject) {
  pthread_mutex_lock(&TangoData::GetInstance().xyzij_mutex);
  float ret_val = TangoData::GetInstance().depth_frame_delta_time;
  pthread_mutex_unlock(&TangoData::GetInstance().xyzij_mutex);
  return ret_val;
}

// Touching GL interface.
JNIEXPORT void JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_startSetCameraOffset(
    JNIEnv*, jobject) {
  if (cam != NULL) {
    cam_start_angle[0] = cam_cur_angle[0];
    cam_start_angle[1] = cam_cur_angle[1];
    cam_start_dist = cam->GetPosition().z;
  }
}

JNIEXPORT void JNICALL
Java_com_elation_tango_pointcloudviewer_TangoJNINative_setCameraOffset(
    JNIEnv*, jobject, float rotation_x, float rotation_y, float dist) {
  if (cam != NULL) {
    cam_cur_angle[0] = cam_start_angle[0] + rotation_x;
    cam_cur_angle[1] = cam_start_angle[1] + rotation_y;
    dist = tango_gl::util::Clamp(cam_start_dist + dist * kZoomSpeed,
                                 kCamViewMinDist, kCamViewMaxDist);
    cam_cur_dist = dist;
  }
}
#ifdef __cplusplus
}
#endif
