// Qt
#include <QtCore\QDir>
#include <QtCore\QDebug>
#include <QtCore\QFileInfo>
#include <QtCore\QUrl>
#include <QtGui\QFileDialog>
#include <QtGui\QMessageBox>
#include <QtGui\QMouseEvent>
// Ogre
#include <OgreEntity.h>
#include <OgreImage.h>
#include <OgreMaterialManager.h>
#include <OgreMeshManager.h>
#include <OgreMeshSerializer.h>
#include <OgreOverlayContainer.h>
#include <OgreOverlayManager.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSubMesh.h>
#include <OgreTextureManager.h>
// Local
#include "MeshWidget.h"


// --------------------------------------------------------
//  public: Constructor
// --------------------------------------------------------
MeshWidget::MeshWidget(QWidget* parent)
  : OgreWidget(parent),
    TEMP_RESOURCE_GROUP("LoadedResources")
{
  mpEntity = NULL;
  mpAnimation = NULL;
  mpYawNode = NULL;
  mpPitchNode = NULL;
  mpCamNode = NULL;
  mpBackgroundRect = NULL;
  mpBackgroundMaterial = NULL;
  mAnimationPaused = false;

  setAcceptDrops(true);
}

// --------------------------------------------------------
//  public: Destructor
// --------------------------------------------------------
MeshWidget::~MeshWidget(void)
{
}

// --------------------------------------------------------
//  public: Load Mesh
// --------------------------------------------------------
void MeshWidget::loadMesh(QString path)
{
  // First, unload the previous mesh
  unloadMesh();

  // Get the filename and directory from the path
  QFileInfo info(path);
  QString filename = info.fileName();
  QString dir = info.dir().path();

  // If it's a valid path
  if (!dir.isNull() && !dir.isEmpty())
  {
    try
    {
      // Add the directory as a resource location
      mpRoot->addResourceLocation(dir.toAscii().data(), "FileSystem", TEMP_RESOURCE_GROUP);
      Ogre::ResourceGroupManager& rm = Ogre::ResourceGroupManager::getSingleton();
      rm.initialiseResourceGroup(TEMP_RESOURCE_GROUP);

      // Check for all needed materials to make sure they exist
      Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().load(filename.toAscii().data(), TEMP_RESOURCE_GROUP);
      if (!mesh.isNull())
      {
        for (int i = 0; i < mesh->getNumSubMeshes(); ++i)
        {
          const Ogre::String& matName = mesh->getSubMesh(i)->getMaterialName();
          if (!Ogre::MaterialManager::getSingleton().resourceExists(matName))
          {
            // Material does not exist.  Let the user navigate to the directoy that contains it
            int r = QMessageBox::warning(this, "Missing Material",
              QString(matName.c_str()) + " could not be found in any of the existing resource group locations.\n" +
              QString("Do you want to search for it?"), "&Yes", "&No", "&Don't Ask Again");

            if (r == 0) // yes
            {
              // Let the user select the directory that contains the material script
              QString matPath = QFileDialog::getExistingDirectory(this, "Locate Material Directory");
              if (!matPath.isNull())
              {
                // Add path to resource locations and initialise
                mpRoot->addResourceLocation(matPath.toAscii().data(), "FileSystem", TEMP_RESOURCE_GROUP);
                rm.initialiseResourceGroup(TEMP_RESOURCE_GROUP);
              }
            }
            else if (r == 1) // no
            {
              // Do nothing
            }
            if (r == 2) // Don't ask again
            {
              // Break out of loop and continue loading the mesh
              break;
            }
          }
        }
      }

      // Load the Entity
      mpEntity = mpsSceneMgr->createEntity("Model", filename.toAscii().data());
      mpsSceneMgr->getRootSceneNode()->attachObject(mpEntity);

      // Reposition the camera based on the bounding box of the entity
      Ogre::Vector3 size = mpEntity->getBoundingBox().getSize();
      Ogre::Vector3 halfSize = mpEntity->getBoundingBox().getHalfSize();
      float maxAxis = std::max<float>(size.x, size.y);
      maxAxis = std::max<float>(maxAxis, size.z);

      // Update the clip boundaries
      mpYawNode->setPosition(mpEntity->getBoundingBox().getCenter());
      mpCamNode->setPosition(0, 0, maxAxis*1.5);
      mpCamNode->lookAt(mpEntity->getBoundingBox().getCenter(), Ogre::Node::TS_WORLD);
      mpCamera->setNearClipDistance(maxAxis * 0.01);
      mpCamera->setFarClipDistance(maxAxis * 100);

      // Update move speed based on distance
      setCameraSpeed(mpCamera->getPosition().length());

      // Now get all animations
      QStringList animations;
      Ogre::AnimationStateSet* animSet = mpEntity->getAllAnimationStates();
      if (animSet)
      {
        Ogre::AnimationStateIterator itr = animSet->getAnimationStateIterator();
        while (itr.hasMoreElements())
        {
          Ogre::AnimationState* state = itr.getNext();
          animations << QString(state->getAnimationName().c_str());
        }
      }
      emit animationsChanged(animations); // let the MainWindow know there are
                                          // new animations
      
    }
    catch (const Ogre::Exception& e)
    {
      QMessageBox::critical(this, "Ogre Exception", QString(e.getFullDescription().c_str()));
    }
  }
}

// --------------------------------------------------------
//  public: Save Mesh
// --------------------------------------------------------
void MeshWidget::saveMesh(QString path)
{
  if (mpEntity)
  {
    Ogre::MeshSerializer serializer;
    serializer.exportMesh(mpEntity->getMesh().getPointer(), path.toAscii().data());
  }
}

// --------------------------------------------------------
//  public: Set Background Image
// --------------------------------------------------------
void MeshWidget::setBackgroundImage(bool enabled, QString imagePath)
{
  // Update visibility
  mpBackgroundRect->setVisible(enabled);

  // If enabled, set the texture
  if (enabled && !imagePath.isNull())
  {
    if (loadTexture(imagePath))
    {
      QString fileName = QFileInfo(imagePath).fileName();

      // Remove all existing TSUs
      mpBackgroundMaterial->getTechnique(0)->getPass(0)
        ->removeAllTextureUnitStates();

      // Add new one
      mpBackgroundMaterial->getTechnique(0)->getPass(0)
        ->createTextureUnitState(fileName.toAscii().data());
    }
  }
}

// --------------------------------------------------------
//  public: Change Background Color (slot)
// --------------------------------------------------------
void MeshWidget::changeBackgroundColor(QColor qColor)
{
  Ogre::ColourValue colour(qColor.redF(), qColor.greenF(), qColor.blueF());
  mpViewport->setBackgroundColour(colour);
}

// --------------------------------------------------------
//  public: Change Animation (slot)
// --------------------------------------------------------
void MeshWidget::changeAnimation(QString name)
{
  // Make sure this is a valid name
  if (!name.isNull() && !name.isEmpty())
  {
    // Make sure this is a valid animation
    if (mpEntity->getAllAnimationStates()->hasAnimationState(name.toAscii().data()))
    {
      Ogre::AnimationState* state = mpEntity->getAnimationState(name.toAscii().data());
      if (state)
      {
        // Check to see if the current animation is playing
        bool playing = false;
        if (mpAnimation)
          playing = mpAnimation->getEnabled();
    
        mpAnimation = state;
        mpAnimation->setEnabled(playing);
      }
    }
  }
}

// --------------------------------------------------------
//  public: Play Animation (slot)
// --------------------------------------------------------
void MeshWidget::playAnimation(void)
{
  if (mpAnimation)
    mpAnimation->setEnabled(true);
  mAnimationPaused = false;
}

// --------------------------------------------------------
//  public: Pause Animation (slot)
// --------------------------------------------------------
void MeshWidget::pauseAnimation(void)
{
  mAnimationPaused = true;
}

// --------------------------------------------------------
//  public: Stop Animation (slot)
// --------------------------------------------------------
void MeshWidget::stopAnimation(void)
{
  if (mpAnimation)
  {
    mpAnimation->setEnabled(false);
    mpAnimation->setTimePosition(0);  // reset time position
  }
  mAnimationPaused = false;
}

// --------------------------------------------------------
//  protected: Init Scene
// --------------------------------------------------------
void MeshWidget::initScene(void)
{
  mpsSceneMgr->setAmbientLight(Ogre::ColourValue(0.125, 0.125, 0.125));

  // Create a Directional Light
  Ogre::Light* light = mpsSceneMgr->createLight("Sun");
  light->setType(Ogre::Light::LT_DIRECTIONAL);
  light->setDirection(Ogre::Vector3(-1, -1, -1));

  // Create background material
  mpBackgroundMaterial = dynamic_cast<Ogre::Material*>(
    Ogre::MaterialManager::getSingleton().create("MeshViewer/Material/Background", "Default").getPointer());
  //mpBackgroundMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("grid.png");
  mpBackgroundMaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);

  // Create rectangle for the background image
  mpBackgroundRect = new Ogre::Rectangle2D(true);
  mpBackgroundRect->setCorners(-1.0, 1.0, 1.0, -1.0);
  mpBackgroundRect->setMaterial("MeshViewer/Material/Background");
  mpBackgroundRect->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
  mpBackgroundRect->setBoundingBox(Ogre::AxisAlignedBox(-100000.0*Ogre::Vector3::UNIT_SCALE, 100000.0*Ogre::Vector3::UNIT_SCALE));
  mpBackgroundRect->setVisible(false);
  mpsSceneMgr->getRootSceneNode()->attachObject(mpBackgroundRect);

}

// --------------------------------------------------------
//  protected: Setup Camera
// --------------------------------------------------------
void MeshWidget::setupCamera(void)
{
  if (mpsSceneMgr && mpWindow)
  {
    // Create the Camera
    mpCamera = mpsSceneMgr->createCamera(mDefaultCameraName);
    mpCamera->setPosition(Ogre::Vector3::ZERO);
    mpCamera->lookAt(0, 0, -1);
    mpCamera->setNearClipDistance(5);
    mpCamera->setAutoAspectRatio(true);
    
    // Setup the camera's SceneNode hiearchy
    mpYawNode = mpsSceneMgr->createSceneNode("Camera/YawNode");
    mpsSceneMgr->getRootSceneNode()->addChild(mpYawNode);

    mpPitchNode = mpsSceneMgr->createSceneNode("Camera/PitchNode");
    mpYawNode->addChild(mpPitchNode);

    mpCamNode = mpsSceneMgr->createSceneNode("Camera/CameraNode");
    mpPitchNode->addChild(mpCamNode);
    mpCamNode->attachObject(mpCamera);

    // Create viewport
    mpViewport = mpWindow->addViewport(mpCamera);
    mpViewport->setBackgroundColour(Ogre::ColourValue(0, 0, 0));

    // Adjust aspect ratio
    float ratio = (float)mpViewport->getActualWidth() / 
      (float)mpViewport->getActualHeight();
    mpCamera->setAspectRatio(ratio);
  }
}

// --------------------------------------------------------
//  protected: Update (virtual)
// --------------------------------------------------------
void MeshWidget::update(float timeSinceLastUpdat)
{
  // Let the OgreWidget do its updates
  OgreWidget::update(timeSinceLastUpdat);

  // Advance the current animation if it's playing
  if (mpAnimation && mpAnimation->getEnabled() && !mAnimationPaused)
    mpAnimation->addTime(timeSinceLastUpdat);
}

// --------------------------------------------------------
//  protected: Unload Mesh
// --------------------------------------------------------
void MeshWidget::unloadMesh(void)
{
  mpAnimation = NULL;
  if (mpEntity)
  {
    // Destroy the entity
    mpEntity->getParentSceneNode()->detachObject(mpEntity);
    mpsSceneMgr->destroyEntity(mpEntity);
    mpEntity = NULL;

    // Unload the temp resource group
    Ogre::ResourceGroupManager& rm = Ogre::ResourceGroupManager::getSingleton();
    rm.unloadResourceGroup(TEMP_RESOURCE_GROUP);
    rm.clearResourceGroup(TEMP_RESOURCE_GROUP);
    rm.removeResourceLocation(TEMP_RESOURCE_GROUP);
  }
}

// ============================================================================
//    QWidget Events
// ============================================================================

// --------------------------------------------------------
//  protected: Paint Event (virtual)
// --------------------------------------------------------
void MeshWidget::paintEvent(QPaintEvent* e)
{
  if (getDC() && !mpRoot)
    if (setup())
      startTimer(25);

  QWidget::paintEvent(e);
}

// --------------------------------------------------------
//  protected: Mouse Press Event (virtual)
// --------------------------------------------------------
void MeshWidget::mousePressEvent(QMouseEvent* e)
{
  if (e->button() == Qt::LeftButton)
  {
    mLastMousePos = e->pos();
  }
}

// --------------------------------------------------------
//  protected: Mouse Move Event (virtual)
// --------------------------------------------------------
void MeshWidget::mouseMoveEvent(QMouseEvent* e)
{
  QPoint diff = e->pos() - mLastMousePos;
  mLastMousePos = e->pos();

  // Left button only
  if (e->buttons() == Qt::LeftButton)
  {
    mRotX = Ogre::Degree(-diff.x()*0.25);
    mRotY = Ogre::Degree(-diff.y()*0.25);

    mpYawNode->yaw(mRotX);
    mpPitchNode->pitch(mRotY);
  }
}

// --------------------------------------------------------
//  protected: Mouse Release Event (virtual)
// --------------------------------------------------------
void MeshWidget::mouseReleaseEvent(QMouseEvent* e)
{
}

// --------------------------------------------------------
//  protected: Key Press Event (virtual)
// --------------------------------------------------------
void MeshWidget::keyPressEvent(QKeyEvent* e)
{
}

// --------------------------------------------------------
//  protected: Key Release Event (virtual)
// --------------------------------------------------------
void MeshWidget::keyReleaseEvent(QKeyEvent* e)
{
}

// --------------------------------------------------------
//  protected: Wheel Event (virtual)
// --------------------------------------------------------
void MeshWidget::wheelEvent(QWheelEvent* e)
{
  float mult = 1.0f;

  if (e->modifiers() & Qt::ShiftModifier)
  {
    // Move slowly
    if (e->delta() < 0)
      mult = 1.05f;
    else if (e->delta() > 0)
      mult = 1/1.05f;
  }
  else
  {
    // Move quickly
    if (e->delta() < 0)
      mult = 1.2f;
    else if (e->delta() > 0)
      mult = 0.8f;
  }

  float dist = mpCamNode->getPosition().z*mult;
  mpCamNode->setPosition(0, 0, dist);
}

// --------------------------------------------------------
//  protected: Drag Enter Event (virtual)
// --------------------------------------------------------
void MeshWidget::dragEnterEvent(QDragEnterEvent* e)
{
  if (e->mimeData()->hasUrls())
    e->acceptProposedAction();
  else
    QWidget::dragEnterEvent(e);
}

// --------------------------------------------------------
//  protected: Drop Event (virtual)
// --------------------------------------------------------
void MeshWidget::dropEvent(QDropEvent* e)
{
  if (e->mimeData()->hasUrls())
  {
    QList<QUrl> urls = e->mimeData()->urls();
    if (!urls.empty())
      loadMesh(urls.first().toLocalFile());
  }
  else
    QWidget::dropEvent(e);
}

// --------------------------------------------------------
//  private: Load Texture
// --------------------------------------------------------
bool MeshWidget::loadTexture(QString path)
{
  bool loaded = false;
  QString fileName = QFileInfo(path).fileName();
  Ogre::TextureManager* manager = Ogre::TextureManager::getSingletonPtr();

  try
  {
    // First see if texture already exists.  If so, we're done
    if (manager->resourceExists(fileName.toAscii().data()))
    {
      loaded = true;
    }
    else
    {
      // Manually load the image.  We use a QImage to load the raw data so that
      // we can let Qt worry about the format and get it into the formate we
      // want.
      QImage qImage(path);
      
      if (!qImage.isNull())
      {
        // Convert to 32-bit RGB
        if (qImage.format() != QImage::Format_RGB32)
          qImage = qImage.convertToFormat(QImage::Format_RGB32);
        
        // Create an Ogre::Image from the QImage
        Ogre::Image image;
        image.loadDynamicImage(
          qImage.bits(), 
          qImage.width(), 
          qImage.height(), 
          Ogre::PF_X8R8G8B8);

        // Create a texture from the image
        Ogre::TexturePtr texture = manager->loadImage(
          fileName.toAscii().data(),
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
          image);

        loaded = !texture.isNull();
      }
    }
  }
  catch (const Ogre::Exception& e)
  {
    QMessageBox::critical(this, "Ogre Exception", QString(e.getFullDescription().c_str()));
    loaded = false;
  }


  return loaded;
}