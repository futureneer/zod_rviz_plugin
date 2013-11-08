/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreOverlayContainer.h>
#include <OGRE/OgreOverlayManager.h>
#include <OGRE/OgreHardwarePixelBuffer.h>

#include <tf/transform_listener.h>

#include <QImage>
#include <QPainter>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include "rviz/properties/ros_topic_property.h"
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/frame_manager.h>

#include "magic_window_visual.h"
#include "magic_window_display.h"

namespace magic_window_rviz_plugin
{

MagicWindowDisplay::MagicWindowDisplay(){}

void MagicWindowDisplay::onInitialize(){

  // Initialize Superclass
  MFDClass::onInitialize();

  // Add Displays
  // scale_property_ = new rviz::VectorProperty( "Scale", Ogre::Vector3(1,1,1),
    // "X and Y Scaling factor for the magic window.  Z will be ignored.", this, SLOT( updateScale() ));

  scale_property_ = new rviz::FloatProperty( "Scale", 1.0,
    "X and Y Scaling factor for the magic window.  Z will be ignored.", this, SLOT( updateScale() ));

  tf_frame_property_ = new rviz::TfFrameProperty( "Attached Frame", "<Fixed Frame>",
    "Tf frame that the magic window follows", this, context_->getFrameManager(), true );

  image_file_property_ = new rviz::StringProperty( "Image File", "",
    "Location of an image file. If an image topic is selected, this will not be used.", this, SLOT( updateImage() ) );

  // Add Image Geometry
  static_visual_.reset(new MagicWindowVisual( context_->getSceneManager(), scene_node_ ));

  ros::NodeHandle nh;
  update_timer_ = nh.createTimer(ros::Duration(0.03), &MagicWindowDisplay::updatePosition, this);
  // ros::Timer timer = nh.createTimer(ros::Duration(0.1), &Foo::callback, &foo_object)
  update_timer_.start();

}

MagicWindowDisplay::~MagicWindowDisplay(){}

// Clear the visuals by deleting their objects.
void MagicWindowDisplay::reset(){
  MFDClass::reset();
  // visuals_.clear();
  update_timer_.stop();
}

void MagicWindowDisplay::updatePosition(const ros::TimerEvent& event){
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  context_->getFrameManager()->getTransform( tf_frame_property_->getStdString(), ros::Time(), position, orientation );
  static_visual_->setFramePosition( position );
  static_visual_->setFrameOrientation( orientation );
}

void MagicWindowDisplay::updateScale(){
  static_visual_->setFrameScale(scale_property_->getFloat());
}

void MagicWindowDisplay::updateImage(){
  static_visual_->updateImage(image_file_property_->getString());
}

// This is our callback to handle an incoming message.
void MagicWindowDisplay::processMessage( const sensor_msgs::Image::ConstPtr& msg ){

  // tf::Transformer* tf = context_->getFrameManager()->getTFClient();
  // if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
  //                                                 msg->header.stamp,
  //                                                 position, orientation ))
  // {
  //   ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
  //              msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  //   return;
  // }

  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full
  // boost::shared_ptr<MagicWindowVisual> visual;
  // if( visuals_.full() )
  // {
  //   visual = visuals_.front();
  // }
  // else
  // {
  //   visual.reset(new MagicWindowVisual( context_->getSceneManager(), scene_node_ ));
  // }

  // Now set or update the contents of the chosen visual.
  // static_visual_->setMessage( msg );

  // float alpha = alpha_property_->getFloat();
  // Ogre::ColourValue color = color_property_->getOgreColor();
  // visual->setColor( color.r, color.g, color.b, alpha );

  // And send it to the end of the circular buffer
  // visuals_.push_back(visual);
}

} // end namespace magic_window_rviz_plugin

// Tell pluginlib about this class.  
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(magic_window_rviz_plugin::MagicWindowDisplay,rviz::Display )

