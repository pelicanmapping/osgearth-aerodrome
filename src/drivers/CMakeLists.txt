SET(TARGET_DEFAULT_PREFIX "osgdb_")

INCLUDE_DIRECTORIES( ${OSG_INCLUDE_DIRS} ../. )

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarth osgEarthFeatures osgEarthAerodrome)

SET(TARGET_SRC
    AerodromeModelSource.cpp
)

SET(TARGET_H
    AerodromeModelSource
)

LINK_INTERNAL(${LIB_NAME}
    osgEarthAerodrome
)

setup_plugin(osgearth_model_aerodrome)

# to install public driver includes:
SET(LIB_NAME model_aerodrome)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
