namespace skeleton_exceptions
{
    class LackingRosParameter : public std::exception {
    private:
        const std::string parameter_;
    public:
    LackingRosParameter(std::string parameter) : parameter_(parameter) {}
        std::string get_info()
        {
            return parameter_;
        }
    };
}
