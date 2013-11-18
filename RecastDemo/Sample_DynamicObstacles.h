#ifndef RECASTSAMPLE_DYNAMICOBSTACLE_H
#define RECASTSAMPLE_DYNAMICOBSTACLE_H

#include "Sample.h"

class Sample_DynamicObstacles : public Sample
{
protected:
    bool m_bKeepInterResults;
    float m_fTotalBuildTimeMs;

    enum DrawMode
    {
        DRAWMODE_NAVMESH,
        MAX_DRAWMODE,
    };

    DrawMode    m_eDrawMode;

public:
    Sample_DynamicObstacles();
    virtual ~Sample_DynamicObstacles();

    virtual void handleSettings();
    virtual void handleTools();
    virtual void handleDebugMode();
    virtual void handleRender();
    virtual void handleRenderOverlay(double* proj, double* model, int* view);
    virtual void handleMeshChanged(class InputGeom* geom);
    virtual bool handleBuild();
    virtual void handleUpdate(const float dt);
};

#endif //RECASTSAMPLE_DYNAMICOBSTACLE_H
