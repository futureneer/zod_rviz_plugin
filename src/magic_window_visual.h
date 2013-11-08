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

#ifndef MAGICWINDOW_VISUAL_H
#define MAGICWINDOW_VISUAL_H

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>


#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreLogManager.h>
#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreMeshManager.h>

namespace Ogre{
class Vector3;
class Quaternion;
}

namespace magic_window_rviz_plugin{

class MagicWindowVisual
{
public:
  MagicWindowVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
  virtual ~MagicWindowVisual();
  void setMessage( const sensor_msgs::Image::ConstPtr& msg );
  void setFrameScale( const Ogre::Vector3& scale);
  void setFrameScale( const float& scale);
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );
  void updateImage(const QString& image_path);
  void updateImage(const QImage& image);
  Ogre::MovablePlane* mPlane;
  Ogre::Entity* mPlaneEnt;
  Ogre::SceneNode* mPlaneNode;
  Ogre::TextureUnitState* tuisTexture;

private:
  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;
};

} // end namespace magic_window_rviz_plugin

#endif // MAGICWINDOW_VISUAL_H
