#include "../PerseusLib/PerseusLib.h"

#include "Utils/Timer.h"
#include <opencv2/opencv.hpp>

using namespace Perseus::Utils;

std::string folder = "/home/src/pwp3d/Files/";

// determine step size relative to current distance to the camera
inline double step_size(double x, double b, double maxval, double k = 8.0)
{
    double diff = fabs(x-b) * k / b + 1;
    double y = maxval / diff;
    if (x > b) y = 2 * maxval - y;
    return y;
}

int main(int argc, char** argv)
{
  std::string sModelPath = argv[argc-11];
  std::string sSrcImage = argv[1];
  std::string sCameraMatrix = folder + "CameraCalibration/drill_n1.cal";
  std::string sTargetMask = folder + "Masks/1280p_All_VideoMask.png";
  std::string sHistSrc = argv[1];
  std::string sHistMask = argv[argc-12];

  
  //  std::string sModelPath = "/Users/luma/Code/Luma/PWP3D/Files/Models/Renderer/long.obj";
  //  std::string sSrcImage = "/Users/luma/Code/Luma/PWP3D/Files/Images/Red.png";
  //  std::string sCameraMatrix = "/Users/luma/Code/Luma/PWP3D/Files/CameraCalibration/900nc.cal";
  //  std::string sTargetMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/480p_All_VideoMask.png";
  //  std::string sHistSrc = "/Users/luma/Code/Luma/PWP3D/Files/Masks/Red_Source.png";
  //  std::string sHistMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/Red_Mask.png";


  // blue car demo
  //  std::string sModelPath = "/Users/luma/Code/DataSet/Mesh/BlueCar.obj";
  //  std::string sSrcImage = "/Users/luma/Code/Luma/PWP3D/Files/Images/248-LiveRGB.png";
  //  std::string sCameraMatrix = "/Users/luma/Code/Luma/PWP3D/Files/CameraCalibration/Kinect.cal";
  //  std::string sTargetMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/480p_All_VideoMask.png";
  //  std::string sHistSrc = "/Users/luma/Code/Luma/PWP3D/Files/Images/248-LiveRGB.png";
  //  std::string sHistMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/248-ID-3-LiveImage.png";

  // red can demo
  //  std::string sModelPath = "/Users/luma/Code/DataSet/Mesh/RedCan.obj";
  //  std::string sSrcImage = "/Users/luma/Code/Luma/PWP3D/Files/Images/248-LiveRGB.png";
  //  std::string sCameraMatrix = "/Users/luma/Code/Luma/PWP3D/Files/CameraCalibration/Kinect.cal";
  //  std::string sTargetMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/480p_All_VideoMask.png";
  //  std::string sHistSrc = "/Users/luma/Code/Luma/PWP3D/Files/Images/248-LiveRGB.png";
  //  std::string sHistMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/248-ID-1-LiveImage.png";

  // ---------------------------------------------------------------------------
  char str[100];
  int i, n = 10;

  int width = 1920, height = 1280;
  int viewCount = 1, objectCount = 1;
  int objectId = 0, viewIdx = 0, objectIdx = 0;
  double step_r, step_xy, step_z, z, param[n];

  Timer t;

  for(i = 0; i < n; i++) param[i] = atof(argv[argc-n+i]);
  
  //result visualisation
  ImageUChar4* ResultImage = new ImageUChar4(width, height);

  // ---------------------------------------------------------------------------
  //input image
  //camera = 24 bit colour rgb
  ImageUChar4* camera = new ImageUChar4(width, height);
  ImageUtils::Instance()->LoadImageFromFile(camera, (char*)sSrcImage.c_str());

  //objects allocation + initialisation: 3d model in obj required
  Object3D **objects = new Object3D*[objectCount];

  std::cout<<"\n==[APP] Init Model =="<<std::endl;
  objects[objectIdx] = new Object3D(objectId, viewCount, (char*)sModelPath.c_str(), width, height);

  // ---------------------------------------------------------------------------
  //views allocation + initialisation: camera calibration (artoolkit format) required
  std::cout<<"\n==[APP] Init CameraMatrix =="<<std::endl;
  View3D **views = new View3D*[viewCount];
  views[viewIdx] = new View3D(0, (char*)sCameraMatrix.c_str(), width, height);


  // ---------------------------------------------------------------------------
  //histogram initialisation
  //source = 24 bit colour rgb
  //mask = 24 bit black/white png - white represents object
  //videoMask = 24 bit black/white png - white represents parts of the image that are usable
  std::cout<<"\n==[APP] Init Target ROI =="<<std::endl;
  ImageUtils::Instance()->LoadImageFromFile(views[viewIdx]->videoMask,
                                            (char*)sTargetMask.c_str());

  ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histSources[viewIdx],
                                            (char*)sHistSrc.c_str());

  ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histMasks[viewIdx],
                                            (char*)sHistMask.c_str(), objectIdx+1);

  HistogramEngine::Instance()->UpdateVarBinHistogram(
        objects[objectIdx], views[viewIdx], objects[objectIdx]->histSources[viewIdx],
        objects[objectIdx]->histMasks[viewIdx], views[viewIdx]->videoMask);


  // ---------------------------------------------------------------------------
  //iteration configuration for one object
  IterationConfiguration *iterConfig = new IterationConfiguration();
  iterConfig->width = width; iterConfig->height = height;
  iterConfig->iterViewIds[viewIdx] = 0;
  iterConfig->iterObjectCount[viewIdx] = 1;
  iterConfig->levelSetBandSize = 8;
  iterConfig->iterObjectIds[viewIdx][objectIdx] = 0;
  iterConfig->iterViewCount = 1;
  iterConfig->iterCount = param[0];
  iterConfig->useCUDAEF = true;
  iterConfig->useCUDARender = true;

  objects[objectIdx]->initialPose[viewIdx]->SetFrom(
        param[1], param[2], param[3], param[4], param[5], param[6]);

  //step size per object and view
  //objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.2f, 0.5f, 0.5f, 10.0f);

  //initial pose per object and view
  // Notice the input pose here is angle, not radians for the rotation part
  //  objects[objectIdx]->initialPose[viewIdx]->SetFrom(
  //        -1.98f, -2.90f, 37.47f, -40.90f, -207.77f, 27.48f);

  // for blue car demo
  //  objects[objectIdx]->initialPose[viewIdx]->SetFrom( -3.0f,-4.5f,28.f, -220.90f, -207.77f, 87.48f);

  // for red can demo
  //  objects[objectIdx]->initialPose[viewIdx]->SetFrom(
  //        1.0f, 3.0f, 30.f, 180.f, 80.f, 60.f);

  //primary initilisation
  OptimisationEngine::Instance()->Initialise(width, height);

  //register camera image with main engine
  OptimisationEngine::Instance()->RegisterViewImage(views[viewIdx], camera);

  // ---------------------------------------------------------------------------
  std::cout<<"\n==[APP] Rendering object initial pose.. =="<<std::endl;
  VisualisationEngine::Instance()->GetImage(
        ResultImage, GETIMAGE_PROXIMITY,
        objects[objectIdx], views[viewIdx],
        objects[objectIdx]->initialPose[viewIdx]);

  cv::Mat ResultMat(height,width,CV_8UC4, ResultImage->pixels);
  cv::imshow("initial pose", ResultMat);
  cv::waitKey(0);

  std::cout<<"[App] Finish Rendered object initial pose."<<std::endl;
  //objects[objectIdx]->histogramVarBin[views[viewIdx]->viewId]->mergeAlphaForeground = 0.03f;
  //objects[objectIdx]->histogramVarBin[views[viewIdx]->viewId]->mergeAlphaBackground = 0.02f;

  bool sequence = argc > 14;
  int loopcount = sequence ? argc-12 : 101;
  for (i=1; i<loopcount; i++)
  {
    z = objects[objectIdx]->initialPose[viewIdx]->translation->z;
    
    step_r = step_size(z, param[3], param[7]);
    step_xy = step_size(z, param[3], param[8]);
    step_z = step_size(z, param[3], param[9]);
    objects[objectIdx]->stepSize[viewIdx]->SetFrom(step_r, step_xy, step_xy, step_z);
    
    printf("%d Step: %f %f %f\n", i, step_r, step_xy, step_z);
    ImageUtils::Instance()->LoadImageFromFile(camera, argv[sequence ? i : 1]);
    OptimisationEngine::Instance()->RegisterViewImage(views[viewIdx], camera);

    //main processing
    t.restart();
    OptimisationEngine::Instance()->Minimise(objects, views, iterConfig);
    objects[objectIdx]->initialPose[viewIdx]->SetFrom(objects[objectIdx]->pose[viewIdx]);
    t.check("Iteration");
    
    //update at your own risk
    //HistogramEngine::Instance()->UpdateVarBinHistogram(
    //    objects[objectIdx], views[viewIdx], camera, objects[objectIdx]->pose[viewIdx]);

    //result plot
    VisualisationEngine::Instance()->GetImage(
          ResultImage, GETIMAGE_PROXIMITY,
          objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

    //result save to file
    sprintf(str, "%sResults/result_%03d.png", folder.c_str(), i);
    //ImageUtils::Instance()->SaveImageToFile(ResultImage, str);
    cv::Mat ResultMat(height,width,CV_8UC4, ResultImage->pixels);
    cv::imshow("result", ResultMat);
    cv::waitKey(1);

    printf("final pose result %f %f %f %f %f %f %f\n\n",
           objects[objectIdx]->pose[viewIdx]->translation->x,
           objects[objectIdx]->pose[viewIdx]->translation->y,
           objects[objectIdx]->pose[viewIdx]->translation->z,
           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.x,
           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.y,
           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.z,
           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.w);

    //posteriors plot
    sprintf(str, "%sResults/posterior_%03d.png", folder.c_str(), i);
    VisualisationEngine::Instance()->GetImage(
            ResultImage, GETIMAGE_POSTERIORS,
            objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

    //ImageUtils::Instance()->SaveImageToFile(ResultImage, str);
  }

  //primary engine destructor
  OptimisationEngine::Instance()->Shutdown();

  for (i = 0; i<objectCount; i++) delete objects[i];
  delete objects;

  for (i = 0; i<viewCount; i++) delete views[i];
  delete views;

  delete ResultImage;

  std::cout<<"Exit pwp3D app successfully."<<std::endl;

  return 0;
}
