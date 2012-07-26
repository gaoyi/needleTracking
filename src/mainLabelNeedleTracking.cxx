/**
 * Find the needles, dark tubes in the MR images
 */

// std
#include <string>

// itk
#include "itkImage.h"

// vtk
#include "vtkSmartPointer.h"
#include "vtkPolyData.h"

// local
#include "utilitiesIO.h"
#include "utilitiesImage.h"

#include "needleTrackingFilter.h"


int main( int argc, char *argv[] )
{
  if( argc < 4 )
    {
      std::cerr << "Usage: inputImage inputLabelImage outputVtk" << std::endl;
    }

  std::string inputImageName(argv[1]);
  std::string inputLabelImageName(argv[2]);
  std::string outputVtkName(argv[3]);

  const unsigned int Dimension = 3;

  typedef double InputPixelType;
  typedef itk::Image< InputPixelType, Dimension >   InputImageType;

  typedef short LabelPixelType;
  typedef itk::Image< LabelPixelType, Dimension >   LabelImageType;

  /// Read input image
  InputImageType::Pointer inputImage = needletracking::readImage<InputImageType>(inputImageName.c_str());

  /// Read label image
  LabelImageType::Pointer labelImage = needletracking::readImage<LabelImageType>(inputLabelImageName.c_str());

  /// Find needles
  typedef needletracking::NeedleTrackingFilter<InputImageType, LabelImageType> NeedleTrackerType;
  NeedleTrackerType* needleTracker = new NeedleTrackerType;
  needleTracker->setInputImage(inputImage);
  needleTracker->setLabelImage(labelImage);
  needleTracker->update();

  needletracking::writePolydataToXMLFile<char>(outputVtkName.c_str(), needleTracker->getNeedlesPolydata());

  delete needleTracker;

  return EXIT_SUCCESS;
}

