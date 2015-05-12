#!/bin/bash

#Copyright 2015 Gregory Kramida

#exit if anything fails
set -e

#-----VARS----------------
NUM_CORES=$(nproc)

#-----FUNCTIONS-----------
package_installed (){
  local __resultvar=$2
  PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $1|grep "install ok installed")
  installed=true
  if [ "" == "$PKG_OK" ]; then
    installed=false
  fi
  eval $__resultvar=${installed}
}

inquire ()  {
   local __resultvar=$2
	while true; do
		 read -p "$1 [Y/n]" yn
		 case $yn in
		     [Yy]* ) yn=1; break;;
		     [Nn]* ) yn=0; break;;
		     * ) echo "Please answer yes or no.";;
		 esac
	done
	eval $__resultvar="$yn"
}

install_ARPG_package_default() {
	local REPO=$1
	local REQUIREMENTS=$2
	local LC_REPO=${REPO,,}
	echo "____________________________________________________________"
	echo ""
	echo "Building & installing ${REPO}."
	echo "____________________________________________________________"
	cd /home/$USER/$NEW_ARPG_DIR/
	mkdir -p builds/${REPO}
	cd builds/${REPO}
	cmake -Wno-dev -G "Eclipse CDT4 - Unix Makefiles" $3 ../../${REPO}
	make -j${NUM_CORES}
	if [ -z "$REQUIREMENTS" ]; then
		sudo checkinstall --exclude=/home -y  --pkgname=${LC_REPO} --conflicts=${LC_REPO}
	else
		sudo checkinstall --exclude=/home -y  --pkgname=${LC_REPO} --conflicts=${LC_REPO} --requires=$REQUIREMENTS
	fi
	sudo ldconfig
	echo "____________________________________________________________"
}
#-------------------------

inquire "Clear out ARPG stuff from /home/$USER/.cmake/packages?" answer
if [ ${answer} == 1 ]; then #begin clear-out block
	
#clear-out .cmake packages
echo "Clearing out ARPG stuff from /home/$USER/.cmake/packages."
echo "____________________________________________________________"
cd /home/$USER/

ARPG_CMAKE_PACKAGES=(Calibu CVARS HAL Kangaroo MINIGLOG Node Pangolin PbMsgs SceneGraph Sophus VICALIB)

if [ -d .cmake/packages ]; then
	for folder in ${ARPG_CMAKE_PACKAGES[@]}; do
		if [ -d .cmake/packages/${folder} ]; then
			echo "Removing old .cmake/packages/${folder}."
			rm -rf .cmake/packages/${folder}
		else
			echo ".cmake/packages/${folder} not found, skipping."
		fi
	done
	echo "Remaining contents: "
	ls .cmake/packages
fi
echo "____________________________________________________________"
fi #end clear-out .cmake ARPG packages block

inquire "Uninstall old ARPG packages?" answer
if [ ${answer} == 1 ]; then #begin uninstall block
echo "____________________________________________________________"
echo ""
echo "Uninstalling old ARPG packages."
echo "____________________________________________________________"

ARPG_PACKAGES=(kangaroo hal node calibu scenegraph pangolin cvars sophus miniglog)
set +e
for pkg in ${ARPG_PACKAGES[@]}; do
	sudo apt-get -y remove ${pkg}
done
set -e
echo "____________________________________________________________"
fi #end uninstall block


ARPG_GIT_REPOS=(miniglog Sophus CVars Pangolin SceneGraph calibu Node HAL vicalib Kangaroo D-MoCap)

inquire "Clear out old ARPG repositores?" answer
if [ ${answer} == 1 ]; then #begin clear-out repo block

echo "____________________________________________________________"
echo ""
echo "Clearing out old ARPG repositories."
echo "____________________________________________________________"

OLD_ARPG_DIR="Garage/ARPG"
found_or_skip=0
skip=0
while [ $found_or_skip == 0 ]; do
	if [ -d /home/$USER/${OLD_ARPG_DIR} ]; then
		found_or_skip=1
	else	
		inquire "Old ARPG location not found at /home/$USER/${OLD_ARPG_DIR}. Would you like to specify an alternative?" answer
		if [ ${answer} == 0 ]; then
			echo "Skipping removal of old ARPG repositories."
			found_or_skip=1
			skip=1
		else
			read -p "Alternative directory with old ARPG repos (relative to /home/$USER/):" OLD_ARPG_DIR
		fi
	fi
done

if [ ${skip} != 1 ]; then
	cd /home/$USER/${OLD_ARPG_DIR}
	for repo in ${ARPG_GIT_REPOS[@]}; do
		if [ -d ${repo} ]; then
			echo "Removing old repo '${repo}'."
			sudo rm -rf ${repo}
		else
			echo "Repo '${repo}' not found, skipping."
		fi
	done
fi
echo "____________________________________________________________"
fi #end clear-out repo block


NEW_ARPG_DIR="Garage/ARPG"
NEW_DEPENDS_DIR="Garage"

response=""
read -p "Directory for ARPG repos, relative to /home/$USER/ [default:Garage/ARPG]:" response
if [ "$response" != "" ]; then
	NEW_ARPG_DIR=${response}
fi
read -p "Directory for ARPG third-party dependencies, relative to /home/$USER/ [default:Garage]:" response
if [ "$response" != "" ]; then
	NEW_DEPENDS_DIR=${response}
fi

if [ ! -d /home/$USER/$NEW_DEPENDS_DIR ]; then
	mkdir -p /home/$USER/$NEW_DEPENDS_DIR
fi

if [ ! -d /home/$USER/$NEW_ARPG_DIR ]; then
	mkdir -p /home/$USER/$NEW_ARPG_DIR
fi


inquire "Checkout current ARPG repositores from github?" answer
if [ ${answer} == 1 ]; then #begin checkout block

echo "____________________________________________________________"
echo ""
echo "Checking out most current ARPG repositories."
echo "____________________________________________________________"

cd /home/$USER/$NEW_ARPG_DIR
remove=0
ignore=0
for repo in ${ARPG_GIT_REPOS[@]}; do
	if [ -d ${repo} ]; then
		if [ $ignore == 0 ]; then

			if [ $remove == 0 ]; then
				inquire "Directory '${repo}'. exists. Remove this and any other conflicting directories and continue?" answer
				if [ ${answer} == 1 ]; then
					sudo rm -rf ${repo}
					remove=1
				else
					inquire "Ignore and treat this and any other conflicting directories as the wanted repositories?" answer
					if [ ${answer} == 1 ]; then
						ignore=1
					else
						"Cannot overwrite git repo '${repo}', exiting."
						exit 1
					fi
				fi
			else
				sudo rm -rf ${repo}
			fi
		fi
	else
		git clone git@github.com:arpg/${repo}.git
	fi
done
echo "____________________________________________________________"
fi #end checkout block


inquire "Install third-party packages from official ubuntu repos? (Recommended for first-time install. Warning: make sure to review the proposed changes, so you don't let this script remove half your freakin' system)." answer
if [ ${answer} == 1 ]; then #begin thrid-party block
echo "____________________________________________________________"
echo ""
echo "Installing third-party packages from official ubuntu repos."
echo "____________________________________________________________"

sudo apt-get install build-essential libtool autoconf libturbojpeg cmake libudev-dev mesa-common-dev freeglut3-dev libxrandr-dev doxygen libxi-dev automake libeigen3-dev libsuitesparse-dev libgoogle-glog-dev libatlas-base-dev libopenexr-dev libtbb-dev libraw1394-dev libpng-dev libjpeg-dev libtiff-dev nvidia-cuda-toolkit libavcodec-ffmpeg-dev libavcodec-ffmpeg-dev libavformat-ffmpeg-dev libavutil-ffmpeg-dev libswscale-ffmpeg-dev qtbase5-dev libzmq3-dev protobuf-compiler libprotoc-dev libxi6 zlib1g-dev libdevil-dev libboost-dev libavahi-compat-libdnssd-dev libgflags2 libgtest-dev checkinstall rsync dos2unix

echo "____________________________________________________________"
fi #end third-party block

inquire "Would you like to build & install ceres-solver (required by ARPG calibu, vicalib)?" answer
if [ ${answer} == 1 ]; then #begin ceres installation block

echo "____________________________________________________________"
echo ""
echo "Building & installing ceres-solver."
echo "____________________________________________________________"

cd /home/$USER/$NEW_DEPENDS_DIR

if [ -d ceres-solver-1.10.0.tar.gz ]; then
	rm -f ceres-solver-1.10.0.tar.gz
fi
if [ -d ceres-solver-1.10.0 ]; then
	echo "Removing existing ceres-solver-1.10.0 folder from /home/$USER/$NEW_DEPENDS_DIR."
	sudo rm -rf ceres-solver-1.10.0
fi

wget http://ceres-solver.org/ceres-solver-1.10.0.tar.gz

tar zxf ceres-solver-1.10.0.tar.gz
rm -f ceres-solver-1.10.0.tar.gz

cd ceres-solver-1.10.0
#build static libs
cmake -G "Eclipse CDT4 - Unix Makefiles" -DEIGENSPARSE=ON -DCMAKE_C_FLAGS=-fPIC -DCMAKE_CXX_FLAGS=-fPIC
make -j${NUM_CORES}

#install package
set +e
sudo dpkg -r ceres-solver
set -e
sudo checkinstall -y --exclude=/home --pkgname=ceres-solver --conflicts=ceres-solver --requires=libeigen3-dev,libsuitesparse-dev,libgoogle-glog-dev,libatlas-base-dev
echo "____________________________________________________________"
fi #end ceres installation block

inquire "Would you like to build & install opencv with opencv_contrib?" answer
if [ ${answer} == 1 ]; then #begin opencv installation block

echo "____________________________________________________________"
echo ""
echo "Building & installing opencv with opencv_contrib."
echo "____________________________________________________________"


cd /home/$USER/$NEW_DEPENDS_DIR

mkdir -p OpenCV
cd OpenCV

if [ -d opencv ]; then
	echo "Removing existing opencv folder from /home/$USER/$NEW_DEPENDS_DIR/OpenCV."
	sudo rm -rf opencv
fi
if [ -d opencv_contrib ]; then
	echo "Removing existing opencv_contrib folder from /home/$USER/$NEW_DEPENDS_DIR/OpenCV."
	sudo rm -rf opencv_contrib
fi



#generic opencv options
echo "This script assumes that Qt5 & GTK development packages are installed... If not, woe be onto you."
cv_options=("-DWITH_1394=ON" "-DWITH_CUBLAS=ON" "-DWITH_CUDA=ON" "-DWITH_CUFFT=ON" "-DWITH_EIGEN=ON" "-DWITH_FFMPEG=ON" "-DWITH_GIGEAPI=ON" "-DWITH_GSTREAMER=ON" "-DWITH_GSTREAMER_0_10=OFF" "-DWITH_GTK=ON" "-DWITH_IPP=OFF" "-DWITH_JASPER=ON" "-DWITH_JPEG=ON" "-DWITH_LIBV4L=ON" "-DWITH_NVCUVID=OFF" "-DWITH_OPENCL=ON" "-DWITH_OPENCLAMDBLAS=OFF" "-DWITH_OPENCLAMDFFT=OFF" "-DWITH_OPENEXR=ON" "-DWITH_OPENGL=ON" "-DWITH_OPENMP=ON" "-DWITH_OPENNI=OFF" "-DWITH_PNG=ON" "-DWITH_PVAPI=ON" "-DWITH_QT=ON" "-DWITH_TBB=ON" "-DWITH_TIFF=ON" "-DWITH_UNICAP=OFF" "-DWITH_V4L=ON" "-DWITH_VTK=OFF" "-DWITH_XIMEA=OFF" "-DWITH_XINE=OFF")

#retrieve supported instruction sets relevant to opencv
INST_SETS_VC2=(AVX AVX2 SSE SSE2 SSE3 SSE41 SSE42 SSSE3)
wget http://pastebin.com/raw.php?i=A8bAuHAP -O opcode.sh
set +e
dos2unix -ascii opcode.sh
chmod +x opcode.sh || :
INST_SETS_OUT=($(./opcode.sh -l))
set -e
rm -f opcode.sh
declare -a INST_SETS
for elem in ${INST_SETS_OUT[@]}; do

	if [ "$elem" == "SSE4_1" ]; then
		to_add=("SSE41")
	elif [ "$elem" == "SSE4_2" ]; then
	   to_add=("SSE42")
	else
		to_add=("$elem")
	fi

	if [[ " ${INST_SETS_VC2[*]} " == *" ${to_add} "* ]]; then
		INST_SETS+=("$to_add")
	fi
done
if [ "$(cat /proc/cpuinfo | egrep "(\W)avx2($|\W)")" != "" ]; then
	INST_SETS+=('AVX2')
fi
echo "CPU instruction sets supported by both this machine and opencv: ${INST_SETS[*]}"
for inst_set in ${INST_SETS[@]}; do
	cv_options+=("-DENABLE_${inst_set}=ON")
done

#CUDA options
echo "This script assumes that your graphics card supports CUDA bin architecture 3.5 ... If not, woe be onto you."
cv_options+=("-DCUDA_ARCH_BIN=3.5") #to speed up compilation
#assign opencv contrib
cv_options+=("-DOPENCV_EXTRA_MODULES_PATH=/home/$USER/$NEW_DEPENDS_DIR/OpenCV/opencv_contrib")

#clone repos
git clone git@github.com:Itseez/opencv.git
git clone git@github.com:Itseez/opencv_contrib.git
cd opencv
git checkout 2.4
cd ..

mkdir -p build2.4
cd build2.4

export CCACHE_DISABLE=1
cmake -G "Eclipse CDT4 - Unix Makefiles" ${cv_options[*]} ../opencv
make -j${NUM_CORES}
export CCACHE_DISABLE=0

set +e
sudo apt-get remove libopencv-* opencv
set -e

sudo checkinstall -y --exclude=/home --pkgname=opencv --conflicts=opencv,libopencv-dev --requires=libeigen3-dev,libopenexr-dev,libtbb-dev,libraw1394-dev,libpng-dev,libjpeg-dev,libtiff-dev,nvidia-cuda-toolkit,libavcodec-ffmpeg-dev,libavcodec-ffmpeg-dev,libavformat-ffmpeg-dev,libavutil-ffmpeg-dev,libswscale-ffmpeg-dev
sudo ldconfig
echo "____________________________________________________________"
fi #end opencv installation block

inquire "Would you like to build & install sodium and zmqpp (ARPG Node requirements)?" answer
if [ ${answer} == 1 ]; then #begin sodium & zmqpp installation block

echo "____________________________________________________________"
echo ""
echo "Building & installing sodium and zmqpp."
echo "____________________________________________________________"

cd /home/$USER/$NEW_DEPENDS_DIR

if [ -d libsodium ]; then
	echo "Removing existing libsodium folder from /home/$USER/$NEW_DEPENDS_DIR."
	sudo rm -rf libsodium
fi

if [ -d zmqpp ]; then
	echo "Removing existing zmqpp folder from /home/$USER/$NEW_DEPENDS_DIR."
	sudo rm -rf zmqpp
fi

set +e
sudo apt-get remove libsodium zmqpp libzmqpp-dev
set -e

git clone git@github.com:jedisct1/libsodium.git
git clone git@github.com:zeromq/zmqpp.git

cd libsodium
./autogen.sh
./configure
make -j${NUM_CORES} check
libsodium_ver=$(cat builds/msvc/version.h | grep -ohP "(?<=SODIUM_VERSION_STRING\s\")[0-9]+\.[0-9]+\.[0-9]+")

sudo checkinstall -y --exclude=/home --pkgversion=${libsodium_ver} --pkgname=libsodium --conflicts=libsodium-dev,libsodium
sudo ldconfig

cd ../zmqpp
make
make -j${NUM_CORES} check
sudo checkinstall -y --exclude=/home --pkgname=zmqpp --conflicts=zmqpp,libzmqpp-dev,libzmqpp3 --requires=libsodium,libzmq3-dev
sudo ldconfig
echo "____________________________________________________________"
fi #end sodium & zmqpp installation block

inquire "Would you like to build & install assimp (used by ARPG SceneGraph, Kangaroo)?" answer
if [ ${answer} == 1 ]; then #begin assimp installation block

echo "____________________________________________________________"
echo ""
echo "Building & installing assimp."
echo "____________________________________________________________"

cd /home/$USER/$NEW_DEPENDS_DIR

if [ -d assimp ]; then
	echo "Removing existing assimp folder from /home/$USER/$NEW_DEPENDS_DIR."
	sudo rm -rf assimp
fi

set +e
sudo apt-get remove -y assimp
set -e

git clone git@github.com:assimp/assimp.git
cd assimp
cmake -G "Eclipse CDT4 - Unix Makefiles" .
make -j${NUM_CORES}
sudo checkinstall -y --exclude=/home --pkgname=assimp --conflicts=assimp --requires=zlib1g-dev
sudo ldconfig

echo "____________________________________________________________"
fi #end assimp installation block


inquire "Would you like to build & install dorian3d (ARPG) version of libfreenect2 (ARPG HAL, Kangaroo requirement for use of Kinect2)?" answer
if [ ${answer} == 1 ]; then #begin libfreenect2 installation block

echo "____________________________________________________________"
echo ""
echo "Building & installing libfreenect2."
echo "____________________________________________________________"
#build in ARPG folder because it's their version of libfreenect2
cd /home/$USER/$NEW_ARPG_DIR/

if [ -d libfreenect2 ]; then
	echo "Removing existing libfreenect2 folder from /home/$USER/$NEW_ARPG_DIR."
	sudo rm -rf libfreenect2
fi

set +e
sudo apt-get remove -y protonect
set -e

git clone git@github.com:dorian3d/libfreenect2.git
cd libfreenect2/depends
sh install_ubuntu.sh
sudo ldconfig

if [ ! -h /usr/lib/x86_64-linux-gnu/libturbojpeg.so ]; then
	#fix turbojpeg if needed
	sudo ln -s /usr/lib/x86_64-linux-gnu/libturbojpeg.so.0.0.0 /usr/lib/x86_64-linux-gnu/libturbojpeg.so
	sudo ldconfig
fi
#------FIX CMAKE FILES---------------#

cd ../examples/protonect

sed -i "s/\"Enable C++11 support\"\s*OFF/\"Enable C++11 support\" ON/" CMakeLists.txt

sed -i "s/ADD_DEFINITIONS(-DGLEW_MX -DGLEW_STATIC)/ADD_DEFINITIONS(-DGLEW_MX -DGLEW_STATIC)\n\nSET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)/" CMakeLists.txt

sed -i "s/SET[(]freenect2_LIBRARIES [$][{]freenect2_LIBRARY[}] [@]LIBFREENECT2_THREADING_LIBRARIES[@][)]/SET(freenect2_LIBRARIES \$\{freenect2_LIBRARY\} @LIBFREENECT2_THREADING_LIBRARIES@ glfw)/" freenect2.cmake.in

sed -i "s/# GLEW/# GLEW\nGET_FILENAME_COMPONENT(GLEW_DIR \"\${MY_DIR}\/..\/..\/depends\/glew\/\" REALPATH)/" CMakeLists.txt

sed -i "s/INCLUDE_DIRECTORIES(\"\${MY_DIR}\/..\/..\/depends\/glew\/include\/\")/INCLUDE_DIRECTORIES(\"\${GLEW_DIR}\/include\/\")/" CMakeLists.txt

sed -i "s/LINK_DIRECTORIES(\"\${MY_DIR}\/..\/..\/depends\/glew\/lib\/\")/LINK_DIRECTORIES(\"\${GLEW_DIR}\/lib\/\")/" CMakeLists.txt

sed -i "s/LINK_DIRECTORIES(\"\${MY_DIR}\/..\/..\/depends\/glew\/lib64\/\")/LINK_DIRECTORIES(\"\${GLEW_DIR}\/lib64\/\")/" CMakeLists.txt

printf "\nSET(freenect2_DEFINITIONS \${freenect2_DEFINITIONS} -DGLEW_MX)" >> freenect2.cmake.in


#------REPLACE LIBUSB-----------------#
#remove link
sudo rm -f /lib/x86_64-linux-gnu/libusb-1.0.so.0
if [ ! -f /lib/x86_64-linux-gnu/libusb-1.0.so.0.1.0_old ];then
	#backup old libusb
	sudo mv /lib/x86_64-linux-gnu/libusb-1.0.so.0.1.0 /home/$USER/libusb-1.0.so.0.1.0_old
fi
sudo cp ../../depends/libusb/lib/libusb-1.0.so.0.1.0 /lib/x86_64-linux-gnu/libusb-1.0.so.0.1.0
#re-add link
sudo ln -s /lib/x86_64-linux-gnu/libusb-1.0.so.0.1.0 /lib/x86_64-linux-gnu/libusb-1.0.so.0
sudo ldconfig

cmake -Wno-dev -G "Eclipse CDT4 - Unix Makefiles"  .
make -j${NUM_CORES}

sudo checkinstall -y --exclude=/home --pkgname=protonect --conflicts=protonect

#check that we're linking to the correct GlewMX
linked_GlewMX=$(ldd /usr/local/lib/libfreenect2.so | grep -o -P "(?<=libGLEWmx.so.1.12 => )[^(]*")

if [ "$linked_GlewMX" != "/home/$USER/$NEW_ARPG_DIR/libfreenect2/depends/glew/lib64/libGLEWmx.so.1.12 " ]; then
	echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	echo "Warning:"
	echo "GlewMX in 'ldd /usr/local/lib/libfreenect2.so'"
	echo "is $linked_GlewMX"
	echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
fi
echo "____________________________________________________________"
fi #end libfreenect2 installation block


#---------------------------COMPILE & INSTALL ARPG PACKAGES-----------------------------------#
#installing miniglog Sophus CVars Pangolin SceneGraph calibu Node HAL vicalib Kangaroo D-MoCap

install_ARPG_package_default miniglog
install_ARPG_package_default Sophus "ceres-solver,libeigen3-dev"
install_ARPG_package_default CVars
install_ARPG_package_default Pangolin "libavcodec-ffmpeg-dev,libavcodec-ffmpeg-dev,libavformat-ffmpeg-dev,libavutil-ffmpeg-dev,libswscale-ffmpeg-dev,freeglut3-dev,libxi6,cvars,libjpeg-dev,libpng12-dev,nvidia-cuda-toolkit"
#copy PangolinConfig.h header if it was left behind by install
cd /home/$USER/$NEW_ARPG_DIR/builds/Pangolin
sudo cp include/pangolin/PangolinConfig.h /usr/local/include/pangolin

install_ARPG_package_default SceneGraph "cvars,libeigen3-dev,assimp,libglew-dev,libjpeg-dev,libpng-dev,zlib1g-dev,libdevil-dev" "-DBUILD_EXAMPLES=OFF"
install_ARPG_package_default calibu "ceres-solver,libeigen3-dev,pangolin,sophus,opencv,zlib1g-dev,nvidia-cuda-toolkit,libgoogle-glog-dev,libboost-dev" "-DBUILD_APPLICATIONS=OFF"
#return to build calibu applications w/o install
cd /home/$USER/$NEW_ARPG_DIR/builds/calibu
cmake -G "Eclipse CDT4 - Unix Makefiles" "-DBUILD_APPLICATIONS=ON" ../../calibu
make -j${NUM_CORES}
install_ARPG_package_default Node "libgoogle-glog-dev,libavahi-compat-libdnssd-dev,libgflags2,protobuf-compiler,libprotoc-dev,libsodium,libzmq3-dev,zmqpp" "-DCMAKE_C_FLAGS=-fPIC -DCMAKE_CXX_FLAGS=-fPIC"
install_ARPG_package_default HAL "protonect,calibu,libeigen3-dev,node,pangolin,sophus,zmqpp"

install_ARPG_package_default vicalib "cvars,calibu,ceres-solver,libeigen3-dev,hal,pangolin,scenegraph,sophus,libgoogle-glog-dev,libgtest-dev,libgflags2,opencv"
install_ARPG_package_default Kangaroo "nvidia-cuda-toolkit,calibu,libeigen3-dev,hal,pangolin,scenegraph,sophus,assimp,opencv" "-DCUDA_npp_LIBRARY=/usr/lib/x86_64-linux-gnu/libnppi.so"

#D-MoCap is currently broken by the latest update to HAL, install manually at your own peril

exit 0
