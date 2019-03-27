#ifndef SKELETON_EXCEPTIONS
#define SKELETON_EXCEPTIONS

/**
 * Define all new exceptions needed for this package.
 */
namespace skeleton_exceptions
{
    /**
     * This exception should be thrown if a required ros parameter has not been provided.
     */
    class LackingRosParameter : public std::exception {
    private:
        const std::string parameter_name_;
    public:
    LackingRosParameter(std::string parameter_name) : parameter_name_(parameter_name) {}
        std::string get_info()
        {
            return parameter_name_;
        }
    };

    class InvalidBodyPartList : public std::exception {
    };
}

#endif
