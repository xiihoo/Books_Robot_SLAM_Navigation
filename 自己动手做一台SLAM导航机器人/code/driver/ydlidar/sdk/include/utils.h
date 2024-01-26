
#pragma once

#ifdef WIN32
#ifdef ydlidar_EXPORTS
#define YDLIDAR_API __declspec(dllexport)
#else
#ifdef ydlidarStatic_EXPORTS
#define YDLIDAR_API 
#else

#define YDLIDAR_API __declspec(dllimport)
#endif // YDLIDAR_STATIC_EXPORTS
#endif

#else 
#define YDLIDAR_API 
#endif // ifdef WIN32
