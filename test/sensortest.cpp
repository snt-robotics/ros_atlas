#include "../src/sensorlistener.h"
#include "helpers.h"

TEST(Sensors, test1)
{
    SensorListener listener;

    // from entity to sensor
    tf2::Transform sensorTransform;
    sensorTransform.setOrigin({ 10, 0, 0 });
    sensorTransform.setRotation({ 0, 0, 0, 1 });

    // from entity to marker
    tf2::Transform entityMarkerTransform;
    entityMarkerTransform.setOrigin({ 2, 0, 0 });
    entityMarkerTransform.setRotation({ 0, 0, 0, 1 });

    // from sensor to marker
    atlas::MarkerData msg;
    msg.pos.x = 5;
    msg.rot.w = 1;
    msg.sigma = 1.0;
    msg.id    = 0;

    listener.onSensorDataAvailable("source", "target", "testSensor", sensorTransform, entityMarkerTransform, msg);
    auto sensorData = listener.filteredSensorData();
    auto result     = sensorData.front().transform.getOrigin();

    // 10 + 5 - 2
    ASSERT_TRUE(vec3Eq({ 13, 0, 0 }, result));
}
