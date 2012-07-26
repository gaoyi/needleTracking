#ifndef utilitiesIO_hxx_
#define utilitiesIO_hxx_

#include <csignal>
#include <string>

// itk
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

// vtk
#include "vtkSmartPointer.h"
#include "vtkPolyData.h"
#include "vtkPolyDataReader.h"
#include "vtkXMLPolyDataWriter.h"

// local
#include "utilitiesIO.h"

namespace needletracking
{
  /**
   * readImage
   */
  template< typename itkImage_t >
  typename itkImage_t::Pointer readImage(const char *fileName)
  {
    typedef itk::ImageFileReader< itkImage_t > ImageReaderType;
    typename ImageReaderType::Pointer reader = ImageReaderType::New();
    reader->SetFileName(fileName);

    typename itkImage_t::Pointer image;
    
    try
      {
        reader->Update();
        image = reader->GetOutput();
      }
    catch ( itk::ExceptionObject &err)
      {
        std::cerr<< "ExceptionObject caught !" << std::endl; 
        std::cerr<< err << std::endl; 
        raise(SIGABRT);
      }

    return image;
  }


  /**
   * writeImage
   */
  template< typename itkImage_t > void writeImage(typename itkImage_t::Pointer img, const char *fileName)
  {
    typedef itk::ImageFileWriter< itkImage_t > WriterType;

    typename WriterType::Pointer writer = WriterType::New();
    writer->SetFileName( fileName );
    writer->SetInput(img);
    writer->UseCompressionOn();

    try
      {
        writer->Update();
      }
    catch ( itk::ExceptionObject &err)
      {
        std::cout << "ExceptionObject caught !" << std::endl; 
        std::cout << err << std::endl; 
        raise(SIGABRT);   
      }
  }

  /********************************************************************************
   * write vtkpolydata 
   */
  template< typename TNull >
  void writePolydataToXMLFile(const char *fileName, vtkSmartPointer<vtkPolyData> pd)
  {
    vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    writer->SetFileName(fileName);
#if VTK_MAJOR_VERSION <= 5
    writer->SetInput(pd);
#else
    writer->SetInputData(pd);
#endif
 
    // Optional - set the mode. The default is binary.
    //writer->SetDataModeToBinary();
    writer->SetDataModeToAscii();
 
    writer->Write();

    return;
  }

  template< typename TNull >
  vtkSmartPointer<vtkPolyData>
  readPolyDataFromVtkFile(const char *fileName)
  {
    vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();
    reader->SetFileName( fileName);
    reader->Update();

    vtkSmartPointer<vtkPolyData> polyData = reader->GetOutput();
    polyData->Update();

    return polyData;  
  }

}// namespace needletracking

#endif
