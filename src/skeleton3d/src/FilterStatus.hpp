#ifndef CLOUD_FILTER_STATUS
#define CLOUD_FILTER_STATUS

namespace pointcloud_filter_status
{
    enum Status {calibrating, ready};
    class CalibrationAborted : public std::exception {};
}

#endif
