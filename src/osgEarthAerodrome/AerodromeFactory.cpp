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
#include "BoundaryNode"
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

template <typename T, typename Y>
void AerodromeFactory::createFeatureNodes(AerodromeFeatureOptions featureOpts, AerodromeContext& context, const osgDB::Options* options)
{
    if (!featureOpts.featureOptions().isSet())
    {
        OE_WARN << LC << "Cannot create feature: feature source is not set." << std::endl;
        return;
    }

    osg::ref_ptr<FeatureSource> featureSource = FeatureSourceFactory::create(featureOpts.featureOptions().value());
    featureSource->initialize(options);
    
    OE_NOTICE << LC << "Reading features...\n";

    int featureCount = 0;

    std::map<std::string, Y*> groupNodes;

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
                Y* parentGroup = 0L;

                std::map<std::string, Y*>::iterator itr = groupNodes.find(icao);
                if (itr != groupNodes.end())
                {
                    parentGroup = itr->second;
                }
                else
                {
                    parentGroup = new Y();
                    groupNodes[icao] = parentGroup;
                    an->addChild(parentGroup);
                }

                if (parentGroup)
                {
                    OE_NOTICE << LC << "Adding feature to aerodrome: " << icao << std::endl;

                    // expand aerodrome bounds
                    if (f->getGeometry())
                        an->bounds().expandBy(f->getGeometry()->getBounds());

                    // create new node and add to parent AerodromeNode
                    parentGroup->addChild(new T(featureOpts, icao, f));
                    featureCount++;
                }
            }            
        }
        else
        {
            OE_WARN << LC << "Skipping feature with empty icao code" << std::endl;
        }
    }

    OE_NOTICE << LC << featureCount << " feature nodes created." << std::endl;
}

void AerodromeFactory::createBoundaryNodes(AerodromeFeatureOptions featureOpts, AerodromeContext& context, const osgDB::Options* options)
{
    if (!featureOpts.featureOptions().isSet())
    {
        OE_WARN << LC << "Cannot create feature: feature source is not set." << std::endl;
        return;
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
                OE_NOTICE << LC << "Adding boundary to aerodrome: " << icao << std::endl;

                // expand aerodrome bounds
                if (f->getGeometry())
                    an->bounds().expandBy(f->getGeometry()->getBounds());

                // create new node and add to parent AerodromeNode
                an->setBoundary(new BoundaryNode(featureOpts, icao, f));
                featureCount++;
            }            
        }
        else
        {
            OE_WARN << LC << "Skipping feature with empty icao code" << std::endl;
        }
    }

    OE_NOTICE << LC << featureCount << " boundary nodes created." << std::endl;
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

    for(AerodromeOptionsSet::const_iterator i = catalog->boundaryOptions().begin(); i != catalog->boundaryOptions().end(); ++i)
        createBoundaryNodes(*i, context, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->pavementOptions().begin(); i != catalog->pavementOptions().end(); ++i)
        createFeatureNodes<PavementNode, PavementGroup>(*i, context, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->taxiwayOptions().begin(); i != catalog->taxiwayOptions().end(); ++i)
        createFeatureNodes<TaxiwayNode, TaxiwayGroup>(*i, context, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->runwayOptions().begin(); i != catalog->runwayOptions().end(); ++i)
        createFeatureNodes<RunwayNode, RunwayGroup>(*i, context, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->runwayThresholdOptions().begin(); i != catalog->runwayThresholdOptions().end(); ++i)
        createFeatureNodes<RunwayThresholdNode, RunwayThresholdGroup>(*i, context, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->stopwayOptions().begin(); i != catalog->stopwayOptions().end(); ++i)
        createFeatureNodes<StopwayNode, StopwayGroup>(*i, context, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->linearFeatureOptions().begin(); i != catalog->linearFeatureOptions().end(); ++i)
        createFeatureNodes<LinearFeatureNode, LinearFeatureGroup>(*i, context, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->startupLocationOptions().begin(); i != catalog->startupLocationOptions().end(); ++i)
        createFeatureNodes<StartupLocationNode, StartupLocationGroup>(*i, context, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->lightBeaconOptions().begin(); i != catalog->lightBeaconOptions().end(); ++i)
        createFeatureNodes<LightBeaconNode, LightBeaconGroup>(*i, context, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->lightIndicatorOptions().begin(); i != catalog->lightIndicatorOptions().end(); ++i)
         createFeatureNodes<LightIndicatorNode, LightIndicatorGroup>(*i, context, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->windsockOptions().begin(); i != catalog->windsockOptions().end(); ++i)
        createFeatureNodes<WindsockNode, WindsockGroup>(*i, context, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->terminalOptions().begin(); i != catalog->terminalOptions().end(); ++i)
        createFeatureNodes<TerminalNode, TerminalGroup>(*i, context, options);

    OE_NOTICE << LC << "Created " << context.aerodromes.size() << " aerodromes." << std::endl;

    return context.root.release();
}