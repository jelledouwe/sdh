/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


// ##################
// #### includes ####
// standard includes
#include <unistd.h>
#include <string>
#include <vector>
#include <cmath>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <schunk_sdh/TactileSensor.h>
#include <schunk_sdh/TactileMatrix.h>
#include <schunk_sdh_ros/ContactInfo.h>
#include <schunk_sdh_ros/ContactInfoArray.h>

// ROS diagnostic msgs
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>

#include <schunk_sdh/dsa.h>

#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>

template<typename T>
  bool read_vector(ros::NodeHandle &n_, const std::string &key, std::vector<T> & res)
  {
    XmlRpc::XmlRpcValue namesXmlRpc;
    if (!n_.hasParam(key))
    {
      return false;
    }

    n_.getParam(key, namesXmlRpc);
    /// Resize and assign of values to the vector
    res.resize(namesXmlRpc.size());
    for (int i = 0; i < namesXmlRpc.size(); i++)
    {
      res[i] = (T)namesXmlRpc[i];
    }
    return true;
  }

/*!
 * \brief Implementation of ROS node for DSA.
 *
 * Offers actionlib and direct command interface.
 */
class DsaNode
{
public:
  /// create a handle for this node, initialize node
  ros::NodeHandle nh_;
private:
  // declaration of topics to publish
  ros::Publisher topicPub_TactileSensor_;
  ros::Publisher topicPub_Diagnostics_;
  ros::Publisher topicPub_ContactInfo_;

  // topic subscribers

  // service servers

  // actionlib server

  // service clients
  // --

  // other variables
  SDH::cDSA *dsa_;
  SDH::UInt32 last_data_publish_;  // time stamp of last data publishing
  SDH::UInt32 last_data_publish_contact_;  // time stamp of last data publishing

  std::string dsadevicestring_;
  std::string dsadevicetype_;
  int dsadevicenum_;
  int maxerror_;  // maximum error count allowed

  bool isDSAInitialized_;
  int error_counter_;
  bool polling_;  // try to publish on each response
  bool auto_publish_;
  bool use_rle_;
  bool debug_;
  double frequency_, timeout_;
  int dsa_port_;
  double dsa_calib_ratio_ = 592.1/ 0.00473;
  double dsa_calib_pressure_ [6];
  double dsa_calib_voltage_ [6];

  ros::Timer timer_dsa, timer_publish, timer_diag;

  std::vector<int> dsa_reorder_;
public:
  /*!
   * \brief Constructor for SdhNode class
   *
   * \param name Name for the actionlib server
   */
  DsaNode() :
      nh_("~"), dsa_(0), last_data_publish_(0), last_data_publish_contact_(0), isDSAInitialized_(false), error_counter_(0)
  {
    topicPub_Diagnostics_ = nh_.advertise < diagnostic_msgs::DiagnosticArray > ("/diagnostics", 1);
    topicPub_TactileSensor_ = nh_.advertise < schunk_sdh::TactileSensor > ("tactile_data", 1);
    topicPub_ContactInfo_ = nh_.advertise < schunk_sdh_ros::ContactInfoArray > ("contact_info_array", 1);
  }

  /*!
   * \brief Destructor for SdhNode class
   */
  ~DsaNode()
  {
    if (isDSAInitialized_)
      dsa_->Close();
    if (dsa_)
      delete dsa_;
  }

  void shutdown()
  {
    timer_dsa.stop();
    timer_publish.stop();
    timer_diag.stop();
    nh_.shutdown();
  }

  /*!
   * \brief Initializes node to get parameters, subscribe and publish to topics.
   */
  bool init()
  {
    // implementation of topics to publish

    nh_.param("dsadevicestring", dsadevicestring_, std::string(""));
    nh_.param("dsadevicetype", dsadevicetype_, std::string(""));
    if (dsadevicestring_.empty())
      return false;

    nh_.param("dsadevicenum", dsadevicenum_, 0);
    nh_.param("maxerror", maxerror_, 8);

    double publish_frequency, diag_frequency;

    nh_.param("debug", debug_, false);
    nh_.param("polling", polling_, false);
    nh_.param("use_rle", use_rle_, true);
    nh_.param("diag_frequency", diag_frequency, 5.0);
    nh_.param("dsaport", dsa_port_, 1300);
    nh_.param("timeout", timeout_, static_cast<double>(0.04));
    nh_.param("dsa_calib_pressure0", dsa_calib_pressure_[0], 0.000473); // unit: N/(mm*mm)
    nh_.param("dsa_calib_pressure1", dsa_calib_pressure_[1], 0.000473); // unit: N/(mm*mm)
    nh_.param("dsa_calib_pressure2", dsa_calib_pressure_[2], 0.000473); // unit: N/(mm*mm)
    nh_.param("dsa_calib_pressure3", dsa_calib_pressure_[3], 0.007014); // unit: N/(mm*mm)
    nh_.param("dsa_calib_pressure4", dsa_calib_pressure_[4], 0.000473); // unit: N/(mm*mm)
    nh_.param("dsa_calib_pressure5", dsa_calib_pressure_[5], 0.028057); // unit: N/(mm*mm)
    nh_.param("dsa_calib_voltage0", dsa_calib_voltage_[0], 592.1);      // unit: mV
    nh_.param("dsa_calib_voltage1", dsa_calib_voltage_[1], 592.1);      // unit: mV
    nh_.param("dsa_calib_voltage2", dsa_calib_voltage_[2], 592.1);      // unit: mV
    nh_.param("dsa_calib_voltage3", dsa_calib_voltage_[3], 932.8);      // unit: mV
    nh_.param("dsa_calib_voltage4", dsa_calib_voltage_[4], 592.1);      // unit: mV
    nh_.param("dsa_calib_voltage5", dsa_calib_voltage_[5], 178.4);      // unit: mV

    frequency_ = 30.0;
    if (polling_)
      nh_.param("poll_frequency", frequency_, 5.0);
    nh_.param("publish_frequency", publish_frequency, 0.0);

    auto_publish_ = true;

    if (polling_)
    {
      timer_dsa = nh_.createTimer(ros::Rate(frequency_).expectedCycleTime(), boost::bind(&DsaNode::pollDsa, this));
    }
    else
    {
      timer_dsa = nh_.createTimer(ros::Rate(frequency_ * 2.0).expectedCycleTime(),
                                  boost::bind(&DsaNode::readDsaFrame, this));
      if (publish_frequency > 0.0)
      {
        auto_publish_ = false;
        timer_publish = nh_.createTimer(ros::Rate(publish_frequency).expectedCycleTime(),
                                        boost::bind(&DsaNode::publishTactileData, this));
      }
    }

    timer_diag = nh_.createTimer(ros::Rate(diag_frequency).expectedCycleTime(),
                                 boost::bind(&DsaNode::publishDiagnostics, this));

    if (!read_vector(nh_, "dsa_reorder", dsa_reorder_))
    {
      dsa_reorder_.resize(6);
      dsa_reorder_[0] = 2;  // t1
      dsa_reorder_[1] = 3;  // t2
      dsa_reorder_[2] = 4;  // f11
      dsa_reorder_[3] = 5;  // f12
      dsa_reorder_[4] = 0;  // f21
      dsa_reorder_[5] = 1;  // f22
    }

    return true;
  }
  bool stop()
  {
    if (dsa_)
    {
      if (isDSAInitialized_)
        dsa_->Close();
      delete dsa_;
    }
    dsa_ = 0;
    isDSAInitialized_ = false;
    return true;
  }

  bool start()
  {
    if (isDSAInitialized_ == false)
    {
      // Init tactile data
      if (dsadevicetype_.compare("TCP") == 0)
      {
        try
        {
		  ROS_INFO("Initializins TCP for DSA Tactile Sensors on device %s", dsadevicestring_.c_str());
          dsa_ = new SDH::cDSA(0, dsadevicestring_.c_str(), dsa_port_, timeout_ );
          if (!polling_)
            dsa_->SetFramerate(frequency_, use_rle_);
          else
            dsa_->SetFramerate(0, use_rle_);

          ROS_INFO("Initialized RS232 for DSA Tactile Sensors on device %s", dsadevicestring_.c_str());
          // ROS_INFO("Set sensitivity to 1.0");
          // for(int i=0; i<6; i++)
          //  dsa_->SetMatrixSensitivity(i, 1.0);
          error_counter_ = 0;
          isDSAInitialized_ = true;
        }
        catch (SDH::cSDHLibraryException* e)
        {
          isDSAInitialized_ = false;
          ROS_ERROR("An exception was caught: %s", e->what());
          delete e;

          shutdown();
          return false;
        }
      }
      else if (!dsadevicestring_.empty())
      {
        try
        {
          dsa_ = new SDH::cDSA(0, dsadevicenum_, dsadevicestring_.c_str());
          if (!polling_)
            dsa_->SetFramerate(frequency_, use_rle_);
          else
            dsa_->SetFramerate(0, use_rle_);

          ROS_INFO("Initialized RS232 for DSA Tactile Sensors on device %s", dsadevicestring_.c_str());
          // ROS_INFO("Set sensitivity to 1.0");
          // for(int i=0; i<6; i++)
          //  dsa_->SetMatrixSensitivity(i, 1.0);
          error_counter_ = 0;
          isDSAInitialized_ = true;
        }
        catch (SDH::cSDHLibraryException* e)
        {
          isDSAInitialized_ = false;
          ROS_ERROR("An exception was caught: %s", e->what());
          delete e;

          shutdown();
          return false;
        }
      }
    }

    return true;
  }

  void readDsaFrame()
  {
    if (debug_)
      ROS_DEBUG("readDsaFrame");

    if (isDSAInitialized_)
    {
      try
      {
        SDH::UInt32 last_time;
        last_time = dsa_->GetFrame().timestamp;
        dsa_->UpdateFrame();
        if (last_time != dsa_->GetFrame().timestamp)
        {
          // new data
          if (error_counter_ > 0)
            --error_counter_;
          if (auto_publish_)
            publishTactileData();
            publishContactData();
        }
      }
      catch (SDH::cSDHLibraryException* e)
      {
        ROS_ERROR("An exception was caught: %s", e->what());
        delete e;
        ++error_counter_;
      }
      if (error_counter_ > maxerror_)
        stop();
    }
    else
    {
      start();
    }
  }

  void pollDsa()
  {
    if (debug_)
      ROS_DEBUG("pollDsa");

    if (isDSAInitialized_)
    {
      try
      {
        dsa_->SetFramerate(0, use_rle_);
        readDsaFrame();
      }
      catch (SDH::cSDHLibraryException* e)
      {
        ROS_ERROR("An exception was caught: %s", e->what());
        delete e;
        ++error_counter_;
      }
      if (error_counter_ > maxerror_)
        stop();
    }
    else
    {
      start();
    }
  }

  void publishTactileData()
  {
    if (debug_)
      ROS_DEBUG("publishTactileData %ul %ul", dsa_->GetFrame().timestamp, last_data_publish_);
    if (!isDSAInitialized_ || dsa_->GetFrame().timestamp == last_data_publish_)
      return;  // no new frame available
    last_data_publish_ = dsa_->GetFrame().timestamp;

    schunk_sdh::TactileSensor msg;
    msg.header.stamp = ros::Time::now();
    int m, x, y;
    double width, height, area;
    msg.tactile_matrix.resize(dsa_->GetSensorInfo().nb_matrices);
    ROS_ASSERT(dsa_->GetSensorInfo().nb_matrices == dsa_reorder_.size());
    for (unsigned int i = 0; i < dsa_reorder_.size(); i++)
    {
      m = dsa_reorder_[i];
      schunk_sdh::TactileMatrix &tm = msg.tactile_matrix[i];
      tm.matrix_id = i;
      tm.cells_x = dsa_->GetMatrixInfo(m).cells_x;
      tm.cells_y = dsa_->GetMatrixInfo(m).cells_y;
      tm.tactile_array.resize(tm.cells_x * tm.cells_y);
      width = dsa_->GetMatrixInfo(m).texel_width;
      height = dsa_->GetMatrixInfo(m).texel_height;
      area = tm.cells_x * tm.cells_y * width * height / pow(10.0, 6);
      for (y = 0; y < tm.cells_y; y++)
      {
        for (x = 0; x < tm.cells_x; x++)
          tm.tactile_array[tm.cells_x * y + x] = dsa_->GetTexel(m, x, y) * area;
      }
    }
    // publish matrix
    topicPub_TactileSensor_.publish(msg);
  }
  void publishContactData()
    {
      if (debug_)
        ROS_DEBUG("publishContactData %ul %ul", dsa_->GetFrame().timestamp, last_data_publish_contact_);
      if (!isDSAInitialized_ || dsa_->GetFrame().timestamp == last_data_publish_contact_)
        return;  // no new frame available
      last_data_publish_contact_ = dsa_->GetFrame().timestamp;

      schunk_sdh_ros::ContactInfoArray msg;
      msg.header.stamp = ros::Time::now();
      SDH::cDSA::sContactInfo sdh_contact_info;
      int m;
      msg.contact_info.resize(dsa_->GetSensorInfo().nb_matrices);
      ROS_ASSERT(dsa_->GetSensorInfo().nb_matrices == dsa_reorder_.size());
      for (unsigned int i = 0; i < dsa_reorder_.size(); i++)
      {
    	m = dsa_reorder_[i];
    	//m = i;
        schunk_sdh_ros::ContactInfo &cf = msg.contact_info[i];
        cf.matrix_id = i;
        sdh_contact_info = dsa_->GetContactInfo(m);
        cf.force = sdh_contact_info.force;
        cf.x_center = sdh_contact_info.cog_x;
    		cf.y_center = sdh_contact_info.cog_y;
    		cf.contact_area = sdh_contact_info.area;
    		cf.in_contact =  (cf.force > 0) ? true : false;
      }
      // publish matrix
      topicPub_ContactInfo_.publish(msg);
    }
  void publishDiagnostics()
  {
    // publishing diagnotic messages
    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.status.resize(1);
    diagnostics.status[0].name = nh_.getNamespace();
    diagnostics.status[0].values.resize(1);
    diagnostics.status[0].values[0].key = "error_count";
    diagnostics.status[0].values[0].value = boost::lexical_cast < std::string > (error_counter_);

    // set data to diagnostics
    if (isDSAInitialized_)
    {
      diagnostics.status[0].level = 0;
      diagnostics.status[0].message = "DSA tactile sensing initialized and running";
    }
    else if (error_counter_ == 0)
    {
      diagnostics.status[0].level = 1;
      diagnostics.status[0].message = "DSA not initialized";
    }
    else
    {
      diagnostics.status[0].level = 2;
      diagnostics.status[0].message = "DSA exceeded eror count";
    }
    // publish diagnostic message
    topicPub_Diagnostics_.publish(diagnostics);
    if (debug_)
      ROS_DEBUG_STREAM("publishDiagnostics " << diagnostics);
  }
};
// DsaNode

/*!
 * \brief Main loop of ROS node.
 *
 * Running with a specific frequency defined by loop_rate.
 */
int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "schunk_dsa");

  DsaNode dsa_node;

  if (!dsa_node.init())
    return 0;

  dsa_node.start();

  ROS_INFO("...dsa node running...");

  ros::spin();

  ROS_INFO("...dsa node shut down...");
  return 0;
}
