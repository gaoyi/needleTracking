#ifndef needleTrackingFilter_hxx_
#define needleTrackingFilter_hxx_

// std
#include <vector>
#include <algorithm>

// itk
#include "itkImage.h"
#include "itkImageRegionIterator.h"
#include "itkImageRegionConstIterator.h"
#include "itkPoint.h"

// vtk
#include "vtkSmartPointer.h"
#include "vtkPolyData.h"
#include "vtkLineSource.h"
#include "vtkConeSource.h"
#include "vtkAppendPolyData.h"
#include "vtkTubeFilter.h"
#include "vtkShortArray.h"
#include "vtkPointData.h"

// local 
#include "needleTrackingFilter.h"
#include "utilitiesImage.h"

namespace needletracking
{

template<typename TInputImageType, typename TLabelImageType>
NeedleTrackingFilter< TInputImageType, TLabelImageType>::NeedleTrackingFilter()
{
    m_sigma = 1.0;
    m_alpha1 = 0.1;
    m_alpha2 = 5.0;

    m_baseLabel = 1;
    m_needleRadius = 1.0;

    m_done = false;
}

template<typename TInputImageType, typename TLabelImageType>
void
NeedleTrackingFilter< TInputImageType, TLabelImageType>::setInputImage(typename TInputImageType::Pointer img)
{
    m_inputImage = img;

    m_inputImageFlipped = flipImageIntensity<InputImageType>( m_inputImage );

    return;
}

template<typename TInputImageType, typename TLabelImageType>
void
NeedleTrackingFilter< TInputImageType, TLabelImageType>::setLabelImage(typename TLabelImageType::Pointer label)
{
    m_inputLabelImage = label;

    return;
}

template<typename TInputImageType, typename TLabelImageType>
void
NeedleTrackingFilter< TInputImageType, TLabelImageType>::update()
{
    _sanityCheck();

    _computeVesselnessImage();

    _extractBaseLabelIndexList();

    _extractNeedleLabels();

    _findAllNeedles();

    m_done = true;

    return;
}

template<typename TInputImageType, typename TLabelImageType>
vtkSmartPointer<vtkPolyData>
NeedleTrackingFilter< TInputImageType, TLabelImageType>::getNeedlesPolydata()
{
    if (!m_done)
    {
        std::cerr<<"Error: not done yet.\n";
        abort();
    }

    return m_needlesPolydata;
}


template<typename TInputImageType, typename TLabelImageType>
void
NeedleTrackingFilter< TInputImageType, TLabelImageType>::_sanityCheck()
{
    if (!m_inputImage)
    {
        std::cerr<<"Error: m_inputImage not set.\n";
        abort();
    }

    if (!m_inputLabelImage)
    {
        std::cerr<<"Error: m_inputLabelImage not set.\n";
        abort();
    }

    if (m_inputLabelImage->GetLargestPossibleRegion() != m_inputImage->GetLargestPossibleRegion())
    {
        std::cerr<<"Error: regions of m_inputLabelImage and m_inputImage not match.\n";
        abort();
    }

    return;
}

template<typename TInputImageType, typename TLabelImageType>
void
NeedleTrackingFilter< TInputImageType, TLabelImageType>::_computeVesselnessImage()
{
    /**
     * Vesselness filtering
     */
    m_vesselnessImage                                                   \
            = needletracking::computerVesselnessImage<InputImageType, FloatingImageType>(m_inputImageFlipped, m_sigma, m_alpha1, m_alpha2);

    return;
}


template<typename TInputImageType, typename TLabelImageType>
void
NeedleTrackingFilter< TInputImageType, TLabelImageType>::_extractBaseLabelIndexList()
{
    long ni = m_inputImage->GetLargestPossibleRegion().GetSize()[0];
    long nj = m_inputImage->GetLargestPossibleRegion().GetSize()[1];
    long nk = m_inputImage->GetLargestPossibleRegion().GetSize()[2];

    IndexType idx = {{0, 0, 0}};

    for (long ik = 0; ik < nk; ++ik)
    {
        idx[2] = ik;
        for (long ij = 0; ij < nj; ++ij)
        {
            idx[1] = ij;
            for (long ii = 0; ii < ni; ++ii)
            {
                idx[0] = ii;

                if (m_inputLabelImage->GetPixel(idx) == m_baseLabel)
                {
                    m_baseLabelIndexList.push_back(idx);
                }
            }
        }
    }
    
    return;
}

/**
   * From the label image, extract the label values corresponding to
   * the needles
   */
template<typename TInputImageType, typename TLabelImageType>
void
NeedleTrackingFilter< TInputImageType, TLabelImageType>::_extractNeedleLabels()
{
    typedef itk::ImageRegionConstIterator<LabelImageType> ImageRegionConstIterator;
    ImageRegionConstIterator citer(m_inputLabelImage, m_inputLabelImage->GetLargestPossibleRegion() );

    for (citer.GoToBegin(); !citer.IsAtEnd(); ++citer)
    {
        LabelPixelType thisLabel = citer.Get();
        if (thisLabel != 0 && thisLabel != m_baseLabel)
        {
            m_needleLabels.push_back(thisLabel);
        }
    }

    std::sort(m_needleLabels.begin(), m_needleLabels.end());
    typename std::vector<LabelPixelType>::iterator vctIt = std::unique(m_needleLabels.begin(), m_needleLabels.end());
    m_needleLabels.resize( vctIt - m_needleLabels.begin() );

    m_numberOfNeedles = m_needleLabels.size();

    return;
}


template<typename TInputImageType, typename TLabelImageType>
void
NeedleTrackingFilter< TInputImageType, TLabelImageType>::_findAllNeedles()
{
    m_needlesPolydata = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
    appendFilter->AddInput(m_needlesPolydata);

    for (short indl = 0; indl < m_numberOfNeedles; ++indl)
    {
        short labelOfThisNeedle = m_needleLabels[indl];

        float needleBaseRAS[3];
        float needleTipRAS[3];

        _findNeedleOfLabel(labelOfThisNeedle, needleBaseRAS, needleTipRAS);

        vtkSmartPointer<vtkPolyData> thisNeedlePolydata = _getNeedlePolydata(needleBaseRAS[0], needleBaseRAS[1], needleBaseRAS[2], \
                                                                             needleTipRAS[0], needleTipRAS[1], needleTipRAS[2], labelOfThisNeedle);


        appendFilter->AddInput(thisNeedlePolydata);
        //        appendFilter->AddInputConnection(tubeFilter->GetOutputPort());
        appendFilter->Update();
    }

    m_needlesPolydata = appendFilter->GetOutput();

    return;
}

template<typename TInputImageType, typename TLabelImageType>
void
NeedleTrackingFilter< TInputImageType, TLabelImageType>::_findNeedleOfLabel(LabelPixelType labelOfThisNeedle, float needleBaseRAS[3], float needleTipRAS[3])
{
    /**
     * One needles has a single label, however, when drawing, there
     * may be several voxels for this needle having the same label. So
     * we find the best among them.
     */

    long ni = m_inputImage->GetLargestPossibleRegion().GetSize()[0];
    long nj = m_inputImage->GetLargestPossibleRegion().GetSize()[1];
    long nk = m_inputImage->GetLargestPossibleRegion().GetSize()[2];

    IndexType idx = {{0, 0, 0}};

    std::vector<IndexType> indexWithThisLabel;
    std::vector<double> pathItgFromPositionsOfThisLabel;
    std::vector<IndexType> bestBaseIndexList;

    for (long ik = 0; ik < nk; ++ik)
    {
        idx[2] = ik;
        for (long ij = 0; ij < nj; ++ij)
        {
            idx[1] = ij;
            for (long ii = 0; ii < ni; ++ii)
            {
                idx[0] = ii;

                if (m_inputLabelImage->GetPixel(idx) == labelOfThisNeedle)
                {
                    indexWithThisLabel.push_back(idx);
                    double maxPathItg = 0.0;
                    IndexType bestBaseIndex = _pathIntegralFromOneVoxelToBase(idx, maxPathItg);

                    pathItgFromPositionsOfThisLabel.push_back(maxPathItg);
                    bestBaseIndexList.push_back(bestBaseIndex);
                }
            }
        }
    }


    long locationOfMaxPathItg = static_cast<long>(max_element(pathItgFromPositionsOfThisLabel.begin(), pathItgFromPositionsOfThisLabel.end()) - pathItgFromPositionsOfThisLabel.begin());

    IndexType bestIndexOfThisLabel = indexWithThisLabel[locationOfMaxPathItg];
    IndexType bestBaseIndex = bestBaseIndexList[locationOfMaxPathItg];

    PointType needleTipLPS;
    m_inputImage->TransformIndexToPhysicalPoint(bestIndexOfThisLabel, needleTipLPS);
    needleTipRAS[0] = -needleTipLPS[0];
    needleTipRAS[1] = -needleTipLPS[1];
    needleTipRAS[2] = needleTipLPS[2];

    PointType needleBaseLPS;
    m_inputImage->TransformIndexToPhysicalPoint(bestBaseIndex, needleBaseLPS);
    needleBaseRAS[0] = -needleBaseLPS[0];
    needleBaseRAS[1] = -needleBaseLPS[1];
    needleBaseRAS[2] = needleBaseLPS[2];

    return;
}


template<typename TInputImageType, typename TLabelImageType>
typename NeedleTrackingFilter< TInputImageType, TLabelImageType>::IndexType
NeedleTrackingFilter< TInputImageType, TLabelImageType>::_pathIntegralFromOneVoxelToBase(IndexType idx, double& maxPathItg)
{
    long nb = m_baseLabelIndexList.size();
    std::vector<double> allPathItg(nb, 0.0);

    for (long i = 0; i < nb; ++i)
    {
        IndexType baseIdx = m_baseLabelIndexList[i];

        allPathItg[i] = _pathIntegralFromOneVoxelToOneVoxel(idx, baseIdx);
    }

    long idxOfMaxPathItg = static_cast<long>(max_element(allPathItg.begin(), allPathItg.end()) - allPathItg.begin());
    
    maxPathItg = allPathItg[idxOfMaxPathItg];

    IndexType bestBaseIndex = m_baseLabelIndexList[idxOfMaxPathItg];

    return bestBaseIndex;
}

template<typename TInputImageType, typename TLabelImageType>
double
NeedleTrackingFilter< TInputImageType, TLabelImageType>::_pathIntegralFromOneVoxelToOneVoxel(IndexType startIdx, IndexType endIdx)
{
    typedef typename IndexType::IndexValueType IndexValueType;

    double pathItg = 0.0;

    float idxDiff[3];
    idxDiff[0] = endIdx[0] - startIdx[0];
    idxDiff[1] = endIdx[1] - startIdx[1];
    idxDiff[2] = endIdx[2] - startIdx[2];

    for (float t = 0; t <= 1.0; t += 0.01)
    {
        IndexType idxOnPath;
        idxOnPath[0] = static_cast<IndexValueType>(startIdx[0] + t*idxDiff[0]);
        idxOnPath[1] = static_cast<IndexValueType>(startIdx[1] + t*idxDiff[1]);
        idxOnPath[2] = static_cast<IndexValueType>(startIdx[2] + t*idxDiff[2]);

        pathItg += m_vesselnessImage->GetPixel(idxOnPath);
    }

    return pathItg;
}


template<typename TInputImageType, typename TLabelImageType>
vtkSmartPointer<vtkPolyData>
NeedleTrackingFilter< TInputImageType, TLabelImageType>::_getNeedlePolydata(double xBase0, double xBase1, double xBase2, double xTip0, double xTip1, double xTip2, short needleLabel)
{
    //return _getNeedlePolydataCone(xBase0, xBase1, xBase2, xTip0, xTip1, xTip2, needleLabel);
//    return _getNeedlePolydataRod(xBase0, xBase1, xBase2, xTip0, xTip1, xTip2, needleLabel);
    return _getNeedlePolydataRodAndConeTip(xBase0, xBase1, xBase2, xTip0, xTip1, xTip2, needleLabel);
}



template<typename TInputImageType, typename TLabelImageType>
vtkSmartPointer<vtkPolyData>
NeedleTrackingFilter< TInputImageType, TLabelImageType>::_getNeedlePolydataCone(double xBase0, double xBase1, double xBase2, double xTip0, double xTip1, double xTip2, short needleLabel)
{
    vtkSmartPointer<vtkConeSource> coneSource = vtkSmartPointer<vtkConeSource>::New();
    coneSource->SetCenter((xTip0 + xBase0)/2.0, (xTip1 + xBase1)/2.0, (xTip2 + xBase2)/2.0);
    coneSource->SetDirection(xTip0 - xBase0, xTip1 - xBase1, xTip2 - xBase2);

    coneSource->SetHeight(sqrt((xTip0 - xBase0)*(xTip0 - xBase0) + (xTip1 - xBase1)*(xTip1 - xBase1) + (xTip2 - xBase2)*(xTip2 - xBase2)));

    coneSource->SetRadius(m_needleRadius);
    coneSource->SetResolution(20);
    coneSource->Update();

    //vtkSmartPointer<vtkPolyData> thisTube = tubeFilter->GetOutput();
    vtkSmartPointer<vtkPolyData> thisTube = coneSource->GetOutput();
    // add color (this label) to this tube
    vtkSmartPointer<vtkShortArray> labelOnTube = vtkSmartPointer<vtkShortArray>::New();
    labelOnTube->SetName("NeedleLabel");
    long np = thisTube->GetNumberOfPoints();
    labelOnTube->SetNumberOfValues(np);
    for (long ip = 0; ip < np; ++ip)
    {
        labelOnTube->SetValue(ip, needleLabel);
    }

    thisTube->GetPointData()->SetScalars(labelOnTube);

    return thisTube;
}

template<typename TInputImageType, typename TLabelImageType>
vtkSmartPointer<vtkPolyData>
NeedleTrackingFilter< TInputImageType, TLabelImageType>::_getNeedlePolydataRod(double xBase0, double xBase1, double xBase2, double xTip0, double xTip1, double xTip2, short needleLabel)
{
    vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
    lineSource->SetPoint1(xBase0, xBase1, xBase2);
    lineSource->SetPoint2(xTip0, xTip1, xTip2);
    lineSource->Update();

    vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
    tubeFilter->SetInputConnection(lineSource->GetOutputPort());
    tubeFilter->SetRadius(m_needleRadius);
    tubeFilter->SetNumberOfSides(50);
    tubeFilter->Update();

    vtkSmartPointer<vtkPolyData> thisTube = tubeFilter->GetOutput();

    // add color (this label) to this tube
    vtkSmartPointer<vtkShortArray> labelOnTube = vtkSmartPointer<vtkShortArray>::New();
    labelOnTube->SetName("NeedleLabel");
    long np = thisTube->GetNumberOfPoints();
    labelOnTube->SetNumberOfValues(np);
    for (long ip = 0; ip < np; ++ip)
    {
        labelOnTube->SetValue(ip, needleLabel);
    }

    thisTube->GetPointData()->SetScalars(labelOnTube);

    return thisTube;
}




template<typename TInputImageType, typename TLabelImageType>
vtkSmartPointer<vtkPolyData>
NeedleTrackingFilter< TInputImageType, TLabelImageType>::_getNeedlePolydataRodAndConeTip(double xBase0, double xBase1, double xBase2, double xTip0, double xTip1, double xTip2, short needleLabel)
{
    // rob to 90%, then 10% is cone


    // rod
    double xPosition0 = xBase0 + 0.9*(xTip0 - xBase0);
    double xPosition1 = xBase1 + 0.9*(xTip1 - xBase1);
    double xPosition2 = xBase2 + 0.9*(xTip2 - xBase2);

    vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
    lineSource->SetPoint1(xBase0, xBase1, xBase2);
    lineSource->SetPoint2(xPosition0, xPosition1, xPosition2);
    lineSource->Update();

    vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
    tubeFilter->SetInputConnection(lineSource->GetOutputPort());
    tubeFilter->SetRadius(m_needleRadius);
    tubeFilter->SetNumberOfSides(50);
    tubeFilter->Update();


    // cone
    vtkSmartPointer<vtkConeSource> coneSource = vtkSmartPointer<vtkConeSource>::New();
    coneSource->SetCenter((xTip0 + xPosition0)/2.0, (xTip1 + xPosition1)/2.0, (xTip2 + xPosition2)/2.0);
    coneSource->SetDirection(xTip0 - xBase0, xTip1 - xBase1, xTip2 - xBase2);
    coneSource->SetHeight(sqrt((xTip0 - xPosition0)*(xTip0 - xPosition0) + (xTip1 - xPosition1)*(xTip1 - xPosition1) + (xTip2 - xPosition2)*(xTip2 - xPosition2)));
    coneSource->SetRadius(m_needleRadius);
    coneSource->SetResolution(50);
    coneSource->Update();



    // combine rod and cone
    vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
    appendFilter->AddInput(tubeFilter->GetOutput());
    appendFilter->AddInput(coneSource->GetOutput());
    appendFilter->Update();

    vtkSmartPointer<vtkPolyData> thisNeedle = appendFilter->GetOutput();


    // add color (this label) to this tube
    vtkSmartPointer<vtkShortArray> labelOnTube = vtkSmartPointer<vtkShortArray>::New();
    labelOnTube->SetName("NeedleLabel");
    long np = thisNeedle->GetNumberOfPoints();
    labelOnTube->SetNumberOfValues(np);
    for (long ip = 0; ip < np; ++ip)
    {
        labelOnTube->SetValue(ip, needleLabel);
    }

    thisNeedle->GetPointData()->SetScalars(labelOnTube);

    return thisNeedle;
}


}// namespace needletracking


#endif
