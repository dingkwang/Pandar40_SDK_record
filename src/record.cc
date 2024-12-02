#include "hesaiLidarSDK.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sys/stat.h>
#include <unistd.h>

int imageNo = 0;
unsigned long imageNoForSave = 0;
unsigned int modnum = 0;
int lidarNo = 0;
unsigned long lidarNoForSave = 0;
FILE* cameraTimestampFile = fopen("camera-timestamp.txt", "w");
FILE* lidarTimestampFile = fopen("lidar-timestamp.txt", "w");
FILE* gpsTimestampFile = fopen("gps-timestamp.txt", "w");

double pandoraToSysTimeGap = 0;
int gpsTimestamp = 0;
std::string saveDirectory = "."; // Default to current directory

void gpsCallback(int timestamp)
{
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
    struct timeval ts;
    gettimeofday(&ts, NULL);
    fprintf(lidarTimestampFile, "%d, %f,%f\n", gpsTimestamp, timestamp, ts.tv_sec + (double)ts.tv_usec / 1000000 - pandoraToSysTimeGap - timestamp);

    // Save the point cloud to a PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& point : cld->points)
    {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        cloud->points.push_back(pcl_point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Generate a unique filename for each PCD file
    std::string filename = saveDirectory + "/pointcloud_" + std::to_string(++lidarNoForSave) + ".pcd";
    pcl::io::savePCDFileBinary(filename, *cloud); // Save in binary format
    printf("Saved point cloud to %s\n", filename.c_str());
}

int main(int argc, char **argv)
{
    // Check for directory argument
    if (argc > 1)
    {
        saveDirectory = argv[1];

        // Check if the directory exists
        struct stat info;
        if (stat(saveDirectory.c_str(), &info) != 0)
        {
            // Directory does not exist; attempt to create it
            if (mkdir(saveDirectory.c_str(), 0777) != 0)
            {
                fprintf(stderr, "Error: Could not create directory '%s'.\n", saveDirectory.c_str());
                return -1;
            }
            printf("Directory '%s' created.\n", saveDirectory.c_str());
        }
        else if (!(info.st_mode & S_IFDIR))
        {
            fprintf(stderr, "Error: '%s' exists but is not a directory.\n", saveDirectory.c_str());
            return -1;
        }
        printf("PCD files will be saved in directory: %s\n", saveDirectory.c_str());
    }
    else
    {
        printf("No directory specified. PCD files will be saved in the current directory.\n");
    }

    HesaiLidarSDK psdk(2368, 10110, 0, std::string("/opt/internal/hesai/Pandar40_SDK/build/Pandar40.csv"), lidarCallback, gpsCallback,
                       HESAI_LIDAR_RAW_DATA_STRCUT_DUAL_RETURN, 40, HESAI_LIDAR_PCL_DATA_TYPE_REDUCED);
    psdk.start();
    while (true)
    {
        sleep(100);
    }
}
