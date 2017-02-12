#include "../src/sensorlistener.h"
#include "helpers.h"

TEST(Sensors, test1)
{
    SensorListener listener;

    tf2::Transform transform;
    transform.setOrigin({ 10, 0, 0 });
    transform.setRotation({ 0, 0, 0, 1 });

    atlas::MarkerData msg;
    msg.pos.x = 5;
    msg.rot.w = 1;
    msg.sigma = 1.0;
    msg.id    = 0;

    listener.onSensorDataAvailable("source", "target", "testSensor", transform, tf2::Transform::getIdentity(), msg);
    auto sensorData = listener.filteredSensorData();
    auto result     = sensorData.front().transform.getOrigin();

    tf2::Vector3 expectedPos = { 15, 0, 0 };

    ASSERT_TRUE(vec3Eq(expectedPos, result));
}
