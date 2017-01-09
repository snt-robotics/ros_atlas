#include "../src/sensorlistener.h"
#include "helpers.h"

TEST(Sensors, test1)
{
    SensorListener listener;

    tf2::Transform transform;
    transform.setOrigin({ 10, 0, 0 });
    transform.setRotation({ 0, 0, 0, 1 });

    atlas::markerdataPtr msg = boost::make_shared<atlas::markerdata>();
    msg->position.x          = 5;
    msg->orientation.w       = 1;
    msg->confidance          = 1.0;
    msg->id                  = 0;

    listener.onSensorDataAvailable("source", "testSensor", transform, msg);
    auto sensorData = listener.filteredSensorData();
    auto result     = sensorData.front().transform.getOrigin();

    tf2::Vector3 expectedPos = { 5, 0, 0 };

    ASSERT_TRUE(vec3Eq(expectedPos, result));
}
