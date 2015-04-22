/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include "AerodromeFactory"
#include "Common"
#include "AerodromeNode"
#include "AerodromeCatalog"
#include "LightBeaconNode"
#include "LightIndicatorNode"
#include "LinearFeatureNode"
#include "PavementNode"
#include "RunwayNode"
#include "RunwayThresholdNode"
#include "StartupLocationNode"
#include "StopwayNode"
#include "TaxiwayNode"
#include "TerminalNode"
#include "WindsockNode"

#include <osgEarthFeatures/Feature>
#include <osgEarthFeatures/FeatureSource>


using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Aerodrome;

#define LC "[AerodromeFactory] "


AerodromeFactory::AerodromeFactory(Map* map)
  : _map(map)
{
    //nop
}

template <typename T>
void AerodromeFactory::createFeatureNodes(AerodromeFeatureOptions featureOpts, AerodromeContext& context, const osgDB::Options* options)
{
    if (!featureOpts.featureOptions().isSet())
    {
        OE_WARN << LC << "Cannot create feature: feature source is not set." << std::endl;
    }

    osg::ref_ptr<FeatureSource> featureSource = FeatureSourceFactory::create(featureOpts.featureOptions().value());
    featureSource->initialize(options);
    
    OE_NOTICE << LC << "Reading features...\n";

    int featureCount = 0;

    osg::ref_ptr<FeatureCursor> cursor = featureSource->createFeatureCursor();
    while ( cursor.valid() && cursor->hasMore() )
    {
        Feature* f = cursor->nextFeature();

        /* **************************************** */
        /* Necessary but not sure why               */

        const SpatialReference* ecefSRS = f->getSRS()->getGeographicSRS()->getECEF();

        /* **************************************** */

        std::string icao = f->getString(featureOpts.icaoAttr().value());
        if (!icao.empty())
        {
            osg::ref_ptr<AerodromeNode> an = context.getOrCreateAerodromeNode(icao);
            if (an.valid())
            {
                // create new node and add to parent AerodromeNode
                OE_NOTICE << LC << "Adding feature to aerodrome: " << icao << std::endl;

                an->addChild(new T(icao, f));
                featureCount++;
            }
        }
        else
        {
            OE_WARN << LC << "Skipping feature with empty icao code" << std::endl;
        }
    }

    OE_NOTICE << LC << featureCount << " feature nodes created." << std::endl;
}

//void
//AerodromeFactory::createRunways(AerodromeFeatureOptions runwayOpts, AerodromeContext &context, const osgDB::Options* options)
//{
//}

osg::Group*
AerodromeFactory::createAerodromes(const URI& uri, const osgDB::Options* options)
{
    osg::ref_ptr<AerodromeCatalog> catalog = AerodromeCatalog::read(uri, options);
    return createAerodromes(catalog, options);
}

osg::Group*
AerodromeFactory::createAerodromes(AerodromeCatalog* catalog, const osgDB::Options* options)
{
    OE_NOTICE << LC << "Creating aerodromes..." << std::endl;

    AerodromeContext context;

    if (catalog->lightBeaconOptions().isSet())
        createFeatureNodes<LightBeaconNode>(catalog->lightBeaconOptions().value(), context, options);

    if (catalog->lightIndicatorOptions().isSet())
        createFeatureNodes<LightIndicatorNode>(catalog->lightIndicatorOptions().value(), context, options);

    if (catalog->linearFeatureOptions().isSet())
        createFeatureNodes<LinearFeatureNode>(catalog->linearFeatureOptions().value(), context, options);

    if (catalog->pavementOptions().isSet())
        createFeatureNodes<PavementNode>(catalog->pavementOptions().value(), context, options);

    if (catalog->runwayOptions().isSet())
        createFeatureNodes<RunwayNode>(catalog->runwayOptions().value(), context, options);

    if (catalog->runwayThresholdOptions().isSet())
        createFeatureNodes<RunwayThresholdNode>(catalog->runwayThresholdOptions().value(), context, options);

    if (catalog->startupLocationOptions().isSet())
        createFeatureNodes<StartupLocationNode>(catalog->startupLocationOptions().value(), context, options);

    if (catalog->stopwayOptions().isSet())
        createFeatureNodes<StopwayNode>(catalog->stopwayOptions().value(), context, options);

    if (catalog->taxiwayOptions().isSet())
        createFeatureNodes<TaxiwayNode>(catalog->taxiwayOptions().value(), context, options);

    if (catalog->terminalOptions().isSet())
        createFeatureNodes<TerminalNode>(catalog->terminalOptions().value(), context, options);

    if (catalog->windsockOptions().isSet())
        createFeatureNodes<WindsockNode>(catalog->windsockOptions().value(), context, options);

    OE_NOTICE << LC << "Created " << context.aerodromes.size() << " aerodromes." << std::endl;

    return context.root.release();
}