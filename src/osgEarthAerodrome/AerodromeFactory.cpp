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


namespace
{
    template <typename T, typename Y> class FeatureNodeFinder : public osg::NodeVisitor
    {
    public:
        FeatureNodeFinder(const std::string& attr, const std::string& value)
          : _attr(attr), _value(value), osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
        {
        }

        void apply(T& node)
        {
            if (node.getFeature()->getString(_attr) == _value)
                _found = &node;
        }

        void apply(osg::Group& node)
        {
            if (dynamic_cast<AerodromeNode*>(&node) || dynamic_cast<Y*>(&node))
                traverse(node);
            else if (dynamic_cast<T*>(&node))
                apply(static_cast<T&>(node));
        }

    public:
        T* foundNode() { return _found.get(); }

    private:
        std::string _attr;
        std::string _value;
        osg::ref_ptr<T> _found;
    };
}


AerodromeFactory::AerodromeFactory(Map* map)
  : _map(map)
{
    //nop
}

template <typename T, typename Y>
void AerodromeFactory::createFeatureNodes(AerodromeFeatureOptions featureOpts, AerodromeContext& context, const osgDB::Options* options, void (AerodromeFactory::*processor)(T* node, AerodromeContext& context))
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

                    // create new node
                    T* tNode = new T(featureOpts, icao, f);

                    // if a processor function is passed in, call it
                    if (processor)
                        (this->*processor)(tNode, context);

                    // add the new node to the parent AerodromeNode
                    parentGroup->addChild(tNode);
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

void AerodromeFactory::createBoundaryNodes(BoundaryFeatureOptions boundaryOpts, AerodromeContext& context, const osgDB::Options* options)
{
    if (!boundaryOpts.featureOptions().isSet())
    {
        OE_WARN << LC << "Cannot create feature: feature source is not set." << std::endl;
        return;
    }

    osg::ref_ptr<FeatureSource> featureSource = FeatureSourceFactory::create(boundaryOpts.featureOptions().value());
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

        std::string icao = f->getString(boundaryOpts.icaoAttr().value());
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
                an->setBoundary(new BoundaryNode(boundaryOpts, icao, f));
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

void AerodromeFactory::processStopwayNode(StopwayNode* stopway, AerodromeContext& context)
{
    if (stopway)
    {
        osg::ref_ptr<AerodromeNode> an = context.getOrCreateAerodromeNode(stopway->icao());
        if (an.valid())
        {
            std::string rwyNum = stopway->getFeature()->getString("rwy_num");
            
            FeatureNodeFinder<RunwayNode, RunwayGroup> finder("rwy_num1", rwyNum);
            an->accept(finder);

            osg::ref_ptr<RunwayNode> runway = finder.foundNode();
            if (!runway.valid())
            {
                FeatureNodeFinder<RunwayNode, RunwayGroup> finder2("rwy_num2", rwyNum);
                an->accept(finder2);
                runway = finder2.foundNode();
            }
            
            if (runway.valid())
                stopway->setReferencePoint(runway->getFeature()->getGeometry()->getBounds().center());
            else
                OE_WARN << LC << "Could not find runway " << rwyNum << " for stopway." << std::endl;
        }
    }
}

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

    for(BoundaryOptionsSet::const_iterator i = catalog->boundaryOptions().begin(); i != catalog->boundaryOptions().end(); ++i)
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
        createFeatureNodes<StopwayNode, StopwayGroup>(*i, context, options, &AerodromeFactory::processStopwayNode);

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