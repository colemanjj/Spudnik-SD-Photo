#include "Particle.h"

void setup()
{
}

void loop()
{
#if PLATFORM_ID != PLATFORM_SPARK_CORE
    if (Mesh.ready())
    {
        Serial.print("ready");
    }
#endif
}
