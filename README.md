# prostate-example
A simple Slicer CLI module implementing a semi-automatic prostate segmentation

First build Slicer according to the instructions:
http://wiki.slicer.org/slicerWiki/index.php/Documentation/Nightly/Developers/Build_Instructions

Then configure this module using CMake.
Make sure that SlicerExecutionModel, ITK and VTK are pointing to the ones in Slicer's build tree.
If you built Slicer in _C:\Slicer-SuperBuild-Release_, then these 3 libraries should be in
_C:\Slicer-SuperBuild-Release\SlicerExecutionModel-build_,
_C:\Slicer-SuperBuild-Release\ITKv4-build_ and
_C:\Slicer-SuperBuild-Release\VTKv6-build_.

After you succesfully build this module,
it will be available in Slicer's module selector under group _Segmentation_.
