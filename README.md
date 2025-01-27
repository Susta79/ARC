# ARC
A Robot Controller

To use Qt:
Set the CMake tools extension "Configure Args" to -DCMAKE_PREFIX_PATH=C:/Qt/5.15.2/msvc2019_64

To use Eigen:
Install eigen using vcpkg. ".\vcpkg install eigen3:x64-windows"
Set the CMake tools extension "Configure Args" to -DEigen3_DIR=C:/dev/vcpkg/packages/eigen3_x64-windows/share/eigen3

## Install Eigen macos
After installing Homebrew, run the following command:
brew install eigen
at the end you will get the path where eigen is installed. In my case:
/usr/local/Cellar/eigen/3.4.0_1