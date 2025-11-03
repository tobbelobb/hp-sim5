#include <GCodes/GCodeFileInfo.h>

#include <algorithm>

void GCodeFileInfo::Init() noexcept
{
    fileSize = 0;
    headerSize = 0;
    lastModifiedTime = 0;
    layerHeight = 0.0f;
    numLayers = 0;
    objectHeight = 0.0f;
    std::fill(std::begin(filamentNeeded), std::end(filamentNeeded), 0.0f);
    printTime = 0;
    simulatedTime = 0;
    numFilaments = 0;
    isValid = false;
    incomplete = false;
    for (auto& thumb : thumbnails)
    {
        thumb.Invalidate();
    }
    generatedBy.GetRef().Clear();
}
