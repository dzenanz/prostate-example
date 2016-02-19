#include "ProstateExampleCLP.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCastImageFilter.h"
#include "itkTriangleMeshToBinaryImageFilter.h"
#include "itkMeshFileReader.h"
#include "itkMeshFileWriter.h"
#include "itkGeodesicActiveContourLevelSetImageFilter.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkCurvatureAnisotropicDiffusionImageFilter.h"
#include "itkGradientMagnitudeRecursiveGaussianImageFilter.h"
#include "itkSigmoidImageFilter.h"
#include "itkSignedMaurerDistanceMapImageFilter.h"
#include "itkFileOutputWindow.h"

#include <vector>
#include <string>

#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkDelaunay3D.h>
#include <vtkDataSetSurfaceFilter.h>
#include "vtkOBJWriter.h"

const unsigned int ImageDimension = 3;
typedef itk::Point<double, ImageDimension> PointType;
typedef itk::Mesh<double, ImageDimension> MeshType;
typedef signed short PixelType;
typedef itk::Image< PixelType, ImageDimension > ImageType;
typedef itk::Image< unsigned char, ImageDimension > LabelImageType;
typedef itk::Image< float, ImageDimension > FloatImageType;


//read Slicer's fiducial CSV
std::vector<PointType> readFiducials(std::string fileName)
{
    std::ifstream centerFile(fileName.c_str());
    std::string line;
    //ignore first 3 lines (comments of fiducials savefile)
    std::getline(centerFile, line);
    std::getline(centerFile, line);
    std::getline(centerFile, line);

    std::vector<PointType> points;
    std::getline(centerFile, line);
    while (!centerFile.eof())
    {
        if (!centerFile.good())
            break;
        PointType p;
        std::stringstream iss(line);

        std::string val;
        std::getline(iss, val, ','); //ignore ID
        for (int col = 0; col < 3; ++col)
        {
            std::getline(iss, val, ',');
            if (!iss.good())
                break;

            std::stringstream convertor(val);
            convertor >> p[col];
        }
        points.push_back(p);
        std::getline(centerFile, line);
    }
    return points;
}

//convert vtkPolyData to itk::Mesh by writing it to a file
MeshType::Pointer vtkPolyData2itkMeshThroughFile(vtkPolyData * polyData)
{
    vtkSmartPointer<vtkOBJWriter> hwriter =
        vtkSmartPointer<vtkOBJWriter>::New();
    //hwriter->SetInputData(polydata);
    hwriter->SetInputData(polyData);
    hwriter->SetFileName("C:\\hull.obj");
    hwriter->Update();

    typedef itk::MeshFileReader< MeshType >  MeshReaderType;
    MeshReaderType::Pointer meshReader = MeshReaderType::New();
    meshReader->SetFileName("C:\\hull.obj");
    meshReader->Update();
    return meshReader->GetOutput();
}

//convert vtkPolyData to itk::Mesh
MeshType::Pointer vtkPolyData2itkMesh(vtkPolyData * polyData)
{
    const unsigned int PointDimension = 3;
    const unsigned int MaxCellDimension = 2;
    MeshType::Pointer  mesh = MeshType::New();

    //copy points
    const unsigned int numberOfPoints = polyData->GetNumberOfPoints();
    vtkPoints * vtkpoints = polyData->GetPoints();
    mesh->GetPoints()->Reserve(numberOfPoints);
    for (unsigned int p = 0; p < numberOfPoints; p++)
    {
        double * apoint = vtkpoints->GetPoint(p);
        mesh->SetPoint(p, MeshType::PointType(apoint));
    }

    //count triangles
    vtkIdType  * cellPoints;
    vtkIdType    numberOfCellPoints;
    unsigned int numberOfTriangles = 0;
    vtkCellArray * polygons = polyData->GetPolys();
    polygons->InitTraversal();
    while (polygons->GetNextCell(numberOfCellPoints, cellPoints))
    {
        if (numberOfCellPoints == 3)
        {
            numberOfTriangles++;
        }
    }
    mesh->GetCells()->Reserve(numberOfTriangles);

    //copy triangles
    typedef MeshType::CellType   CellType;
    typedef itk::TriangleCell< CellType > TriangleCellType;
    int cellId = 0;
    polygons->InitTraversal();
    while (polygons->GetNextCell(numberOfCellPoints, cellPoints))
    {
        if (numberOfCellPoints != 3) // skip any non-triangle.
        {
            continue;
        }
        MeshType::CellAutoPointer c;
        TriangleCellType * t = new TriangleCellType;
        //t->SetPointIds((vtkIdType*)cellPoints);
        t->SetPointId(0, cellPoints[0]);
        t->SetPointId(1, cellPoints[1]);
        t->SetPointId(2, cellPoints[2]);
        c.TakeOwnership(t);
        mesh->SetCell(cellId, c);
        cellId++;
    }

    return mesh;
}

int main(int argc, char *argv[])
{
    PARSE_ARGS;
    //extra ; for automatic code formatting

    itk::FileOutputWindow::Pointer win=itk::FileOutputWindow::New();
    win->SetFileName("C:\\itkOutput.txt");
    win->SetInstance(win);

    //read the input image and points
    std::vector<PointType> points;
    typedef itk::ImageFileReader< ImageType  > ImageReaderType;
    ImageReaderType::Pointer  imageReader = ImageReaderType::New();
    imageReader->SetFileName(inputImage);
    try
    {
        imageReader->Update();
        points = readFiducials(initializationPoints);
    }
    catch (itk::ExceptionObject & err)
    {
        std::cerr << "ExceptionObject caught !" << std::endl;
        std::cerr << err << std::endl;
        return EXIT_FAILURE;
    }
    ImageType::ConstPointer image = imageReader->GetOutput();

    //create convex hull
    vtkSmartPointer<vtkPoints> vPoints = vtkSmartPointer<vtkPoints>::New();
    for (int i = 0; i < points.size(); i++)
    {
        vtkIdType id = vPoints->InsertNextPoint(points[i].GetDataPointer());
    }

    vtkSmartPointer< vtkPolyData> polydata =
        vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(vPoints);

    vtkSmartPointer<vtkDelaunay3D> delaunay =
        vtkSmartPointer< vtkDelaunay3D >::New();
    delaunay->SetInputData(polydata);
    delaunay->Update();

    vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter =
        vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
    surfaceFilter->SetInputConnection(delaunay->GetOutputPort());
    surfaceFilter->Update();

    //MeshType::Pointer mesh = vtkPolyData2itkMeshThroughFile(surfaceFilter->GetOutput());
    MeshType::Pointer mesh = vtkPolyData2itkMesh(surfaceFilter->GetOutput());

    //debug
    //typedef itk::MeshFileWriter<MeshType> MeshWriterType;
    //MeshWriterType::Pointer meshWriter = MeshWriterType::New();
    //meshWriter->SetInput(mesh);
    //meshWriter->SetFileName("C:\\mesh.obj");
    //meshWriter->Update();
    
    //voxelize the polyhedron
    typedef itk::CastImageFilter< ImageType, LabelImageType > CastFilterType;
    CastFilterType::Pointer cast = CastFilterType::New();
    cast->SetInput(image);
    typedef itk::TriangleMeshToBinaryImageFilter< MeshType, LabelImageType > FilterType;
    FilterType::Pointer voxelizer = FilterType::New();
    voxelizer->SetInput(mesh);
    voxelizer->SetInfoImage(cast->GetOutput());
    try
    {
        voxelizer->Update();
    }
    catch (itk::ExceptionObject & error)
    {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }
    LabelImageType::Pointer polyhedron = voxelizer->GetOutput();   
    //later use binarized polyhedron as initial image for geodesic active contour
    //the rest is adapted from ITK segmentation example GeodesicActiveContourImageFilter
    //http://itk.org/ITKExamples/src/Segmentation/LevelSets/SegmentWithGeodesicActiveContourLevelSet/Documentation.html

    typedef itk::ImageFileWriter< LabelImageType >  LabelWriterType;
    LabelWriterType::Pointer labelWriter = LabelWriterType::New();
    labelWriter->SetInput(polyhedron);
    labelWriter->SetFileName("C:\\polyhedron.nrrd");
    labelWriter->Update();


    //create speed image using anisotropic diffusion, gradient and sigmoid filters
    typedef   itk::CurvatureAnisotropicDiffusionImageFilter<
        ImageType, FloatImageType > SmoothingFilterType;
    typedef   itk::GradientMagnitudeRecursiveGaussianImageFilter<
        FloatImageType, FloatImageType > GradientFilterType;
    typedef   itk::SigmoidImageFilter<
        FloatImageType, FloatImageType > SigmoidFilterType;

    SmoothingFilterType::Pointer smoothing = SmoothingFilterType::New();
    GradientFilterType::Pointer  gradientMagnitude = GradientFilterType::New();
    SigmoidFilterType::Pointer sigmoid = SigmoidFilterType::New();

    smoothing->SetInput(image);
    gradientMagnitude->SetInput(smoothing->GetOutput());
    sigmoid->SetInput(gradientMagnitude->GetOutput());
    
    smoothing->SetTimeStep(0.0125);
    smoothing->SetNumberOfIterations(5);
    smoothing->SetConductanceParameter(9.0);
    
    gradientMagnitude->SetSigma(1.0);

    sigmoid->SetOutputMinimum(0.0);
    sigmoid->SetOutputMaximum(1.0);
    sigmoid->SetAlpha(-50);
    sigmoid->SetBeta(300);
    sigmoid->Update();

    ////intermediate debug writer
    //typedef itk::ImageFileWriter< FloatImageType >  FloatWriterType;
    //FloatWriterType::Pointer floatWriter = FloatWriterType::New();
    //floatWriter->SetFileName("C:\\smoothing.nrrd");
    //floatWriter->SetInput(smoothing->GetOutput());
    //floatWriter->Update();

    //floatWriter->SetFileName("C:\\gradientMagnitude.nrrd");
    //floatWriter->SetInput(gradientMagnitude->GetOutput());
    //floatWriter->Update();

    //floatWriter->SetFileName("C:\\sigmoid.nrrd");
    //floatWriter->SetInput(sigmoid->GetOutput());
    //floatWriter->Update();

    //level-set segmentation
    typedef itk::SignedMaurerDistanceMapImageFilter<
        LabelImageType, FloatImageType> MaurerType;
    MaurerType::Pointer maurer = MaurerType::New();
    maurer->SetInput(polyhedron);
    maurer->Update();

    //floatWriter->SetFileName("C:\\maurer.nrrd");
    //floatWriter->SetInput(maurer->GetOutput());
    //floatWriter->Update();

    typedef  itk::GeodesicActiveContourLevelSetImageFilter<
        FloatImageType, FloatImageType > GeodesicActiveContourFilterType;
    GeodesicActiveContourFilterType::Pointer geodesicActiveContour =
        GeodesicActiveContourFilterType::New();
    geodesicActiveContour->SetInput(maurer->GetOutput());
    geodesicActiveContour->SetFeatureImage(sigmoid->GetOutput());

    geodesicActiveContour->SetPropagationScaling(2.0);
    geodesicActiveContour->SetCurvatureScaling(3.0);
    geodesicActiveContour->SetAdvectionScaling(1.0);
    geodesicActiveContour->SetMaximumRMSError(rmsError);
    geodesicActiveContour->SetNumberOfIterations(maxIterations);
    geodesicActiveContour->Update(); //the main computation step

    // Print out some useful information
    std::cout << std::endl;
    std::cout << "Max. no. iterations: " << geodesicActiveContour->GetNumberOfIterations() << std::endl;
    std::cout << "Max. RMS error: " << geodesicActiveContour->GetMaximumRMSError() << std::endl;
    std::cout << std::endl;
    std::cout << "No. elpased iterations: " << geodesicActiveContour->GetElapsedIterations() << std::endl;
    std::cout << "RMS change: " << geodesicActiveContour->GetRMSChange() << std::endl;

    //binarize the level set at 0
    typedef itk::BinaryThresholdImageFilter<
        FloatImageType, LabelImageType > ThresholdingFilterType;
    ThresholdingFilterType::Pointer thresholder
        = ThresholdingFilterType::New();
    thresholder->SetInput(geodesicActiveContour->GetOutput());
    thresholder->SetUpperThreshold(0.0);
    thresholder->SetOutsideValue(0);
    thresholder->SetInsideValue(1);
    thresholder->Update();
    LabelImageType::Pointer result = thresholder->GetOutput();

    //write the result
    typedef itk::ImageFileWriter< LabelImageType >  WriterType;
    WriterType::Pointer writer = WriterType::New();
    writer->SetFileName(segmentationResult);
    writer->SetInput(result);
    try
    {
        writer->Update();
    }
    catch (itk::ExceptionObject & err)
    {
        std::cerr << "ExceptionObject caught !" << std::endl;
        std::cerr << err << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
