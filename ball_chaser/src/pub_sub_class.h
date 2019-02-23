#include <ros/ros.h>
#include <string>

class SubscribeAndPublish
{
    public:
        SubscribeAndPublish(std::string SUBSCRIBED_TOPIC, int SUBQSIZE, 
                std::string PUBLISHED_MESSAGE_TYPE, std::string PUBLISHED_TOPIC, int PUBQSIZE)
        {
            //Topic you want to publish
            pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>(PUBLISHED_TOPIC, QSIZE);

            //Topic you want to subscribe 
            sub_ = n_.subscribe(SUBSCRIBED_TOPIC, QSIZE, &SubscribeAndPublish::callback, this);
        }

        void callback(const SUBSCRIBED_MESSAGE_TYPE& input)
        {
            PUBLISHED_MESSAGE_TYPE output:
            // ... do something with the input and generate the output...
            pub_.publish(output);
        }

    private:
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::Subscriber sub_;

}; //End of class SubscribeAndPublish

