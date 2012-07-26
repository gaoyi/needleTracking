#ifndef needleTrackingFilter_h_
#define needleTrackingFilter_h_


#include <string>

// itk
#include "itkImage.h"

// vtk
#include "vtkSmartPointer.h"
#include "vtkPolyData.h"

namespace needletracking
{

  template<typename TInputImageType, typename TLabelImageType>
  class NeedleTrackingFilter
  {
  public:
    NeedleTrackingFilter(); 
    ~NeedleTrackingFilter() {}

    typedef TInputImageType InputImageType;
    typedef TLabelImageType LabelImageType;

    typedef typename LabelImageType::PixelType LabelPixelType;

    typedef typename InputImageType::IndexType IndexType;
    typedef typename InputImageType::PointType PointType;

    typedef double FloatingPixelType;
    typedef itk::Image<FloatingPixelType, InputImageType::ImageDimension> FloatingImageType;

    // fn
    void setInputImage(typename InputImageType::Pointer img);
    void setLabelImage(typename LabelImageType::Pointer label);

    vtkSmartPointer<vtkPolyData> getNeedlesPolydata();

    void update();

  private:

    /// data
    typename InputImageType::Pointer m_inputImage;
    typename InputImageType::Pointer m_inputImageFlipped;
    typename LabelImageType::Pointer m_inputLabelImage;

    typename FloatingImageType::Pointer m_vesselnessImage;

    vtkSmartPointer<vtkPolyData> m_needlesPolydata;

    bool m_done;

    // for vesselness filtering
    double m_sigma;
    double m_alpha1;
    double m_alpha2;

    LabelPixelType m_baseLabel; // label indicating the template base
    std::vector<IndexType> m_baseLabelIndexList;

    std::vector<LabelPixelType> m_needleLabels;

    short m_numberOfNeedles;

    double m_needleRadius;// in mm

    /// fn
    void _sanityCheck();
    void _computeVesselnessImage();

    void _extractBaseLabelIndexList();
    void _extractNeedleLabels();

    void _findAllNeedles();

    void _findNeedleOfLabel(LabelPixelType labelOfThisNeedle, float needleBaseRAS[3], float needleTipRAS[3]);
    IndexType _pathIntegralFromOneVoxelToBase(IndexType idx, double& maxPathItg);
    double _pathIntegralFromOneVoxelToOneVoxel(IndexType startIdx, IndexType endIdx);

    vtkSmartPointer<vtkPolyData> _getNeedlePolydata(double xBase0, double xBase1, double xBase2, double xTip0, double xTip1, double xTip2, short needleLabel );
    vtkSmartPointer<vtkPolyData> _getNeedlePolydataCone(double xBase0, double xBase1, double xBase2, double xTip0, double xTip1, double xTip2, short needleLabel );
    vtkSmartPointer<vtkPolyData> _getNeedlePolydataRod(double xBase0, double xBase1, double xBase2, double xTip0, double xTip1, double xTip2, short needleLabel );
    vtkSmartPointer<vtkPolyData> _getNeedlePolydataRodAndConeTip(double xBase0, double xBase1, double xBase2, double xTip0, double xTip1, double xTip2, short needleLabel );
  };


}// namespace needletracking


#include "needleTrackingFilter.hxx"

#endif
