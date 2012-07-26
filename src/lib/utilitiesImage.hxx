#ifndef utilitiesImage_hxx_
#define utilitiesImage_hxx_

#include <csignal>

// itk
#include "itkMinimumMaximumImageCalculator.h"
#include "itkImageDuplicator.h"
#include "itkCastImageFilter.h"
#include "itkHessianRecursiveGaussianImageFilter.h"
#include "itkHessian3DToVesselnessMeasureImageFilter.h"

#include "itkImage.h"

// local
#include "utilitiesImage.h"

namespace needletracking
{
  /**
   * castItkImage
   */
  template< typename InputImageType, typename OutputImageType > 
  typename OutputImageType::Pointer
  castItkImage( typename InputImageType::Pointer inputImage )
  {
    typedef itk::CastImageFilter< InputImageType, OutputImageType > itkCastFilter_t;
    typename itkCastFilter_t::Pointer caster = itkCastFilter_t::New();
    caster->SetInput( inputImage );
    caster->Update();

    return caster->GetOutput();
  }


  /**
   * Compute the Vesselness image
   */
  template< typename InputImageType, typename OutputImageType > 
  typename OutputImageType::Pointer
  computerVesselnessImage( typename InputImageType::Pointer inputImage, double sigma, double alpha1, double alpha2 )
  {
    /**
     * Hessian filter
     */
    typedef itk::HessianRecursiveGaussianImageFilter< InputImageType > HessianFilterType;
    typename HessianFilterType::Pointer hessianFilter = HessianFilterType::New();
    hessianFilter->SetInput( inputImage );
    hessianFilter->SetSigma( sigma );
    hessianFilter->Update();

    /**
     * Vesselness filtering
     */
    typedef itk::Hessian3DToVesselnessMeasureImageFilter< typename OutputImageType::PixelType > VesselnessMeasureFilterType;
    typename VesselnessMeasureFilterType::Pointer vesselnessFilter = VesselnessMeasureFilterType::New();
    vesselnessFilter->SetInput( hessianFilter->GetOutput() );
    vesselnessFilter->SetAlpha1( alpha1 );
    vesselnessFilter->SetAlpha2( alpha2 );
    vesselnessFilter->Update();

    return vesselnessFilter->GetOutput();
  }

  /**
   * Flip image intensity
   */
  template< typename ImageType > 
  typename ImageType::Pointer
  flipImageIntensity( typename ImageType::Pointer inputImage)
  {
    /**
     * Clone the input image
     */
    typedef itk::ImageDuplicator< ImageType > DuplicatorType;
    typename DuplicatorType::Pointer duplicator = DuplicatorType::New();
    duplicator->SetInputImage(inputImage);
    duplicator->Update();

    typename ImageType::Pointer cloneImage = duplicator->GetOutput();

    /**
     * Compute largest value of the image
     */
    typedef itk::MinimumMaximumImageCalculator<ImageType> ImageCalculatorFilterType;
    typename ImageCalculatorFilterType::Pointer imageCalculatorFilter = ImageCalculatorFilterType::New ();
    imageCalculatorFilter->SetImage(cloneImage);
    imageCalculatorFilter->Compute();

    /**
     * Flip input grayscale
     */
    typedef itk::ImageRegionIterator<ImageType> ImageRegionIterator;
    ImageRegionIterator iter(cloneImage, cloneImage->GetLargestPossibleRegion() );
    typename ImageType::PixelType m = imageCalculatorFilter->GetMaximum();

    for (iter.GoToBegin(); !iter.IsAtEnd(); ++iter)
      {
        iter.Set(m - iter.Get() );
      }

    return cloneImage;
  }


}// namespace needletracking

#endif
