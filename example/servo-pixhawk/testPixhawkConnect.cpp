

#include <iostream>

#include <visp3/core/vpConfig.h>

// Check if std:c++17 or higher
#if defined(VISP_HAVE_MAVSDK) && ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))

#include <visp3/robot/vpRobotMavsdk.h>

void usage(const std::string &bin_name)
{
  std::cerr << "Usage : " << bin_name << " <connection information>\n"
    << "Connection information format should be :\n"
    << "  - For TCP: tcp://[server_host][:server_port]\n"
    << "  - For UDP: udp://[bind_host][:bind_port]\n"
    << "  - For Serial: serial:///path/to/serial/dev[:baudrate]\n"
    << "For example, to connect to the simulator use URL: udp://:14540\n";
}

int main(int argc, char **argv)
{
  std::cout << "argc: " <<argc<< std::endl;
  std::cout << "argv[0]: " <<argv[0]<< std::endl;
  std::cout << "argv[1]: " <<argv[1]<< std::endl;
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  if (argc != 2) {
    usage(argv[0]);
    return EXIT_SUCCESS;
  }

  auto drone = vpRobotMavsdk(argv[1]);

  return EXIT_SUCCESS;
}

#else

int main()
{
#ifndef VISP_HAVE_MAVSDK
  std::cout << "\nThis example requires mavsdk library. You should install it, configure and rebuid ViSP.\n"
    << std::endl;
#endif
#if !((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))
  std::cout
    << "\nThis example requires at least cxx17. You should enable cxx17 during ViSP configuration with cmake and "
    "rebuild ViSP.\n"
    << std::endl;
#endif
  return EXIT_SUCCESS;
}

#endif // #if defined(VISP_HAVE_MAVSDK)
