# tools used in the library
case "$OSTYPE" in
  linux*)
    echo "OS: linux";
    sudo apt update;
    sudo apt install libeigen3-dev;
    sudo apt-get install libfftw3-dev libfftw3-doc;
    sudo apt install libyaml-cpp-dev;
    sudo apt install libspdlog-dev;
    sudo apt install clang-format;;
  darwin*)
    echo "OS: mac"; 
    brew update;
    brew install eigen;
    brew install fftw3;
    brew install yaml-cpp;
    brew install spdlog;
    brew install clang-format;;
  msys*)
    echo "OS: windows";;
  solaris*)
    echo "OS: solaris";;
  bsd*)
    echo "OS: bsd";;
  *)
    echo "OS: unknown";;
esac

# clone repos into utils folder
mkdir src/utils
git clone git@github.com:sturdivant-sdr/file_operations.git src/utils/file_operations
git clone git@github.com:sturdivant-sdr/fftw_wrapper.git src/utils/fftw_wrapper
