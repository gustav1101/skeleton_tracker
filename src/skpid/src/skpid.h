#include <vector>


class ImageConverter
{
private:
    struct BodyPart {
        unsigned int ID;
        float pos_x;
        float pos_y;
    };

    

    static const std::string OPENCV_WINDOW;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber sub;

    static const int RES_X = 640;
    static const int RES_Y = 480;
    
    
    static void findBodyParts(const tfpose_ros::Persons& msg);


public:
    ImageConverter();
    ~ImageConverter();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    static std::vector<BodyPart> last_recognised_bodyparts;
};
