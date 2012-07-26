#ifndef utilitiesIO_h_
#define utilitiesIO_h_


#include <string>

// itk
#include "itkAffineTransform.h"
#include "itkImage.h"

// vtk
#include "vtkSmartPointer.h"
#include "vtkPolyData.h"

namespace needletracking
{
  /**********************************************************************************
   * readImage
   */
  template< typename itkImage_t >
  typename itkImage_t::Pointer readImage(const char *fileName);

  /************************************************************************************
   * writeImage
   */
  template< typename itkImage_t > void writeImage(typename itkImage_t::Pointer img, const char *fileName);


  /********************************************************************************
   * write vtkpolydata 
   */
  template< typename TNull >
  void writePolydataToXMLFile(const char *fileName, vtkSmartPointer<vtkPolyData> pd);

  /********************************************************************************
   * read vtkpolydata 
   */
  template< typename TNull >
  vtkSmartPointer<vtkPolyData>
  readPolyDataFromVtkFile(const char *fileName);

}// namespace needletracking


#include "utilitiesIO.hxx"

#endif
