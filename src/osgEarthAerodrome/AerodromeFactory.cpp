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
#include "TaxiwaySignNode"
#include "TerminalNode"
#include "WindsockNode"
#include "AerodromeRenderer"

#include <osgEarthFeatures/Feature>
#include <osgEarthFeatures/FeatureSource>

#include <osgEarth/Registry>

#include <osg/PagedLOD>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>


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


// -----------------------------------------------------------------------------
// pseudo-loader for paging in aerodromes.

namespace
{
    UID                               _uid         = 0;
    Threading::ReadWriteMutex         _amfMutex;
    typedef std::map<UID, osg::observer_ptr<AerodromeFactory> > AMFRegistry;
    AMFRegistry _amfRegistry;

    static std::string s_makeURI( UID uid, const std::string& icao ) 
    {
        std::stringstream buf;
        buf << uid << "." << icao << ".osgearth_pseudo_amf";
        std::string str;
        str = buf.str();
        return str;
    }

    static std::string s_makeQuery(const std::string& field, const std::string& icao ) 
    {
        std::stringstream buf;
        buf << field << "='" << icao << "'";
        std::string str;
        str = buf.str();
        return str;
    }
}


/**
 * A pseudo-loader for paged feature tiles.
 */
struct osgEarthAerodromeModelPseudoLoader : public osgDB::ReaderWriter
{
    osgEarthAerodromeModelPseudoLoader()
    {
        supportsExtension( "osgearth_pseudo_amf", "Aerodrome model pseudo-loader" );
    }

    const char* className()
    { // override
        return "osgEarth Aerodrome Model Pseudo-Loader";
    }

    ReadResult readNode(const std::string& uri, const Options* options) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension(uri) ) )
            return ReadResult::FILE_NOT_HANDLED;

        UID uid;
        char icao[11];
        sscanf( uri.c_str(), "%u.%10[^'.'].%*s", &uid, icao );

        osg::ref_ptr<AerodromeFactory> factory = getFactory(uid);
        if ( factory.valid() )
        {
            Registry::instance()->startActivity(uri);

            AerodromeNode* node = factory->getAerodromeNode(std::string(icao));

            Registry::instance()->endActivity(uri);

            if (node)
                return ReadResult(node);
        }

        return ReadResult::ERROR_IN_READING_FILE;
    }

    static UID registerFactory( AerodromeFactory* factory )
    {
        Threading::ScopedWriteLock lock( _amfMutex );
        UID key = ++_uid;
        _amfRegistry[key] = factory;
        OE_DEBUG << "Registered AMF " << key << std::endl;
        return key;
    }

    static void unregisterFactory( UID uid )
    {
        Threading::ScopedWriteLock lock( _amfMutex );
        _amfRegistry.erase( uid );
        OE_DEBUG << "Unregistered AMF " << uid << std::endl;
    }

    static AerodromeFactory* getFactory( UID uid ) 
    {
        Threading::ScopedReadLock lock( _amfMutex );
        AMFRegistry::const_iterator i = _amfRegistry.find( uid );
        return i != _amfRegistry.end() ? i->second.get() : 0L;
    }
};

REGISTER_OSGPLUGIN(osgearth_pseudo_amf, osgEarthAerodromeModelPseudoLoader);


osg::ref_ptr<AerodromeRenderer> AerodromeFactory::s_renderer = 0L;

AerodromeFactory::AerodromeFactory(const Map* map, AerodromeCatalog* catalog, const osgDB::Options* options)
  : _map(map), _catalog(catalog), _lodRange(50000.0f)
{
    init(options);
}

AerodromeFactory::AerodromeFactory(const Map* map, AerodromeCatalog* catalog, float lodRange, const osgDB::Options* options)
  : _map(map), _catalog(catalog), _lodRange(lodRange)
{
    init(options);
}

void
AerodromeFactory::init(const osgDB::Options* options)
{
    _uid = osgEarthAerodromeModelPseudoLoader::registerFactory( this );

    _dbOptions = new osgDB::Options( *options );
    //_dbOptions->setObjectCacheHint( osgDB::Options::CACHE_IMAGES );

    // create and initialize a renderer
    _renderer = s_renderer.valid() ? s_renderer : new AerodromeRenderer();
    _renderer->initialize(_map, _dbOptions);

    // setup the PagedLODs
    seedAerodromes(_catalog, _dbOptions);
}

AerodromeFactory::~AerodromeFactory()
{
    osgEarthAerodromeModelPseudoLoader::unregisterFactory( _uid );
}

template <typename T, typename Y, typename P>
void AerodromeFactory::createFeatureNodes(P featureOpts, AerodromeNode* aerodrome, const osgDB::Options* options, void (*processor)(T* node, AerodromeNode* aerodrome))
{
    if (!featureOpts.featureOptions().isSet())
    {
        OE_WARN << LC << "Cannot create features: feature source is not set." << std::endl;
        return;
    }

    if (!aerodrome)
    {
        OE_WARN << LC << "Cannot create features: AerodromeNode is not set." << std::endl;
        return;
    }

    Y* parentGroup = new Y();
    aerodrome->addChild(parentGroup);

    osg::ref_ptr<FeatureSource> featureSource = FeatureSourceFactory::create(featureOpts.featureOptions().value());
    featureSource->initialize(options);

    OE_DEBUG << LC << "Reading features...\n";

    int featureCount = 0;

    Query query;
    query.expression() = s_makeQuery(featureOpts.icaoAttr().value(), aerodrome->icao());

    osg::ref_ptr<FeatureCursor> cursor = featureSource->createFeatureCursor(query);
    while ( cursor.valid() && cursor->hasMore() )
    {
        Feature* f = cursor->nextFeature();

        /* **************************************** */
        /* Necessary but not sure why               */

        const SpatialReference* ecefSRS = f->getSRS()->getGeographicSRS()->getECEF();

        /* **************************************** */

        OE_DEBUG << LC << "Adding feature to aerodrome: " << aerodrome->icao() << std::endl;

        // create new node
        //T* tNode = new T(featureOpts, aerodrome->icao(), featureSource, f->getFID());
        T* tNode = new T(featureOpts, aerodrome->icao(), f);

        // if a processor function is passed in, call it
        if (processor)
            (*processor)(tNode, aerodrome);

        // add the new node to the parent AerodromeNode
        parentGroup->addChild(tNode);
        featureCount++;
    }

    OE_DEBUG << LC << "Added " << featureCount << " feature nodes to aerodrome " << aerodrome->icao() << std::endl;
}



void AerodromeFactory::createBoundaryNodes(BoundaryFeatureOptions boundaryOpts, AerodromeNode* aerodrome, const osgDB::Options* options)
{
    if (!boundaryOpts.featureOptions().isSet())
    {
        OE_WARN << LC << "Cannot create boundary features: feature source is not set." << std::endl;
        return;
    }

    if (!aerodrome)
    {
        OE_WARN << LC << "Cannot create boundary features: AerodromeNode is not set." << std::endl;
        return;
    }

    osg::ref_ptr<FeatureSource> featureSource = FeatureSourceFactory::create(boundaryOpts.featureOptions().value());
    featureSource->initialize(options);
    
    Query query;
    query.expression() = s_makeQuery(boundaryOpts.icaoAttr().value(), aerodrome->icao());

    osg::ref_ptr<FeatureCursor> cursor = featureSource->createFeatureCursor(query);
    while ( cursor.valid() && cursor->hasMore() )
    {
        Feature* f = cursor->nextFeature();

        /* **************************************** */
        /* Necessary but not sure why               */

        const SpatialReference* ecefSRS = f->getSRS()->getGeographicSRS()->getECEF();

        /* **************************************** */

        OE_DEBUG << LC << "Adding boundary to aerodrome: " << aerodrome->icao() << std::endl;

        // create new node and add to parent AerodromeNode
        aerodrome->setBoundary(new BoundaryNode(boundaryOpts, aerodrome->icao(), f));
        break;
    }
}

void AerodromeFactory::processStopwayNode(StopwayNode* stopway, AerodromeNode* aerodrome)
{
    if (stopway)
    {
        std::string rwyNum = stopway->getFeature()->getString("rwy_num");
            
        FeatureNodeFinder<RunwayNode, RunwayGroup> finder("rwy_num1", rwyNum);
        aerodrome->accept(finder);

        osg::ref_ptr<RunwayNode> runway = finder.foundNode();
        if (!runway.valid())
        {
            FeatureNodeFinder<RunwayNode, RunwayGroup> finder2("rwy_num2", rwyNum);
            aerodrome->accept(finder2);
            runway = finder2.foundNode();
        }
            
        if (runway.valid())
            stopway->setReferencePoint(runway->getFeature()->getGeometry()->getBounds().center());
        else
            OE_WARN << LC << "Could not find runway " << rwyNum << " for stopway." << std::endl;
    }
}

AerodromeNode*
AerodromeFactory::createAerodrome(AerodromeCatalog* catalog, const std::string& icao, const osgDB::Options* options)
{
    osg::ref_ptr<AerodromeNode> aerodrome = new AerodromeNode(icao);

    for(BoundaryOptionsSet::const_iterator i = catalog->boundaryOptions().begin(); i != catalog->boundaryOptions().end(); ++i)
        AerodromeFactory::createBoundaryNodes(*i, aerodrome, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->pavementOptions().begin(); i != catalog->pavementOptions().end(); ++i)
        AerodromeFactory::createFeatureNodes<PavementNode, PavementGroup, AerodromeFeatureOptions>(*i, aerodrome, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->taxiwayOptions().begin(); i != catalog->taxiwayOptions().end(); ++i)
        AerodromeFactory::createFeatureNodes<TaxiwayNode, TaxiwayGroup, AerodromeFeatureOptions>(*i, aerodrome, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->runwayOptions().begin(); i != catalog->runwayOptions().end(); ++i)
        AerodromeFactory::createFeatureNodes<RunwayNode, RunwayGroup, AerodromeFeatureOptions>(*i, aerodrome, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->runwayThresholdOptions().begin(); i != catalog->runwayThresholdOptions().end(); ++i)
        AerodromeFactory::createFeatureNodes<RunwayThresholdNode, RunwayThresholdGroup, AerodromeFeatureOptions>(*i, aerodrome, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->stopwayOptions().begin(); i != catalog->stopwayOptions().end(); ++i)
        AerodromeFactory::createFeatureNodes<StopwayNode, StopwayGroup, AerodromeFeatureOptions>(*i, aerodrome, options, &AerodromeFactory::processStopwayNode);

    for(AerodromeOptionsSet::const_iterator i = catalog->linearFeatureOptions().begin(); i != catalog->linearFeatureOptions().end(); ++i)
        AerodromeFactory::createFeatureNodes<LinearFeatureNode, LinearFeatureGroup, AerodromeFeatureOptions>(*i, aerodrome, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->startupLocationOptions().begin(); i != catalog->startupLocationOptions().end(); ++i)
        AerodromeFactory::createFeatureNodes<StartupLocationNode, StartupLocationGroup, AerodromeFeatureOptions>(*i, aerodrome, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->lightBeaconOptions().begin(); i != catalog->lightBeaconOptions().end(); ++i)
        AerodromeFactory::createFeatureNodes<LightBeaconNode, LightBeaconGroup, AerodromeFeatureOptions>(*i, aerodrome, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->lightIndicatorOptions().begin(); i != catalog->lightIndicatorOptions().end(); ++i)
         AerodromeFactory::createFeatureNodes<LightIndicatorNode, LightIndicatorGroup, AerodromeFeatureOptions>(*i, aerodrome, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->taxiwaySignOptions().begin(); i != catalog->taxiwaySignOptions().end(); ++i)
        AerodromeFactory::createFeatureNodes<TaxiwaySignNode, TaxiwaySignGroup, AerodromeFeatureOptions>(*i, aerodrome, options);

    for(AerodromeOptionsSet::const_iterator i = catalog->windsockOptions().begin(); i != catalog->windsockOptions().end(); ++i)
        AerodromeFactory::createFeatureNodes<WindsockNode, WindsockGroup, AerodromeFeatureOptions>(*i, aerodrome, options);

    for(TerminalOptionsSet::const_iterator i = catalog->terminalOptions().begin(); i != catalog->terminalOptions().end(); ++i)
        AerodromeFactory::createFeatureNodes<TerminalNode, TerminalGroup, TerminalFeatureOptions>(*i, aerodrome, options);

    return aerodrome.release();
}

AerodromeNode* AerodromeFactory::getAerodromeNode(const std::string& icao)
{
    if (!_renderer.valid())
        return 0L;

    _mutex.writeLock();

    // create AerodromeNode
    osg::ref_ptr<AerodromeNode> node = createAerodrome(_catalog, icao, _dbOptions);

    // render
    if (_renderer.valid())
        node->accept(*_renderer.get());

    _mutex.writeUnlock();

    return node.release();
}

void 
AerodromeFactory::setDefaultRenderer(AerodromeRenderer* renderer)
{
    s_renderer = renderer;
}

void
AerodromeFactory::seedAerodromes(AerodromeCatalog* catalog, const osgDB::Options* options)
{
    removeChildren(0, getNumChildren());

    OE_DEBUG << LC << "Seeding aerodromes from boundaries." << std::endl;

    int aeroCount = 0;

    for(BoundaryOptionsSet::const_iterator i = catalog->boundaryOptions().begin(); i != catalog->boundaryOptions().end(); ++i)
    {
        if (!i->featureOptions().isSet())
        {
            OE_WARN << LC << "Skipping boundary source: feature source is not set." << std::endl;
            continue;
        }

        osg::ref_ptr<FeatureSource> featureSource = FeatureSourceFactory::create(i->featureOptions().value());
        featureSource->initialize(options);

        osg::ref_ptr<FeatureCursor> cursor = featureSource->createFeatureCursor();
        while ( cursor.valid() && cursor->hasMore() )
        {
            Feature* f = cursor->nextFeature();

            std::string icao = f->getString(i->icaoAttr().value());
            if (!icao.empty())
            {
                // create PagedLOD for this aerodrome
                std::string uri = s_makeURI( _uid, icao );

                if (f->getGeometry())
                {
                    osg::PagedLOD* p = new osg::PagedLOD();
                    p->setFileName(0, uri);

                    GeoPoint gp(f->getSRS(), f->getGeometry()->getBounds().center());
                    osg::Vec3d center;
                    gp.toWorld(center);
                    p->setCenter(center);
                    p->setRadius(std::max((float)f->getGeometry()->getBounds().radius(), _lodRange));
                    p->setRange(0, 0.0f, _lodRange);

                    addChild(p);

                    aeroCount++;
                }
                else
                {
                    OE_DEBUG << LC << "Skipping boundary feature: no geometry." << std::endl;
                }
            }
        }
    }

    OE_DEBUG << LC << aeroCount << " aerodromes found and seeded." << std::endl;
}