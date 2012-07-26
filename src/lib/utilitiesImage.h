#ifndef utilitiesImage_h_
#define utilitiesImage_h_

// itk
#include "itkImage.h"


namespace needletracking
{
  /**
   * Cast image pixel type
   */
  template< typename InputImageType, typename OutputImageType > 
  typename OutputImageType::Pointer
  castItkImage( typename InputImageType::Pointer inputImage );

  /**
   * Compute the Vesselness image
   */
  template< typename InputImageType, typename OutputImageType > 
  typename OutputImageType::Pointer
  computerVesselnessImage( typename InputImageType::Pointer inputImage, double sigma, double alpha1 = 0.1, double alpha2 = 5.0 );

  /**
   * Flip image intensity
   */
  template< typename ImageType > 
  typename ImageType::Pointer
  flipImageIntensity( typename ImageType::Pointer inputImage);


}// namespace needletracking


#include "utilitiesImage.hxx"

#endif
