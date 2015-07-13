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

#include "AerodromeModelGraph"
#include "Common"
#include "AerodromeNode"
#include "AerodromeRenderer"

#include <osgEarth/Registry>

#include <osg/PagedLOD>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>

using namespace osgEarth;
using namespace osgEarth::Aerodrome;

#define LC "[AerodromeModelGraph] "



// -----------------------------------------------------------------------------
// pseudo-loader for paging in aerodromes.

namespace
{
    UID                               _uid         = 0;
    Threading::ReadWriteMutex         _amgMutex;
    typedef std::map<UID, osg::observer_ptr<AerodromeModelGraph> > AMGRegistry;
    AMGRegistry _amgRegistry;

    static std::string s_makeURI( UID uid, const std::string& icao ) 
    {
        std::stringstream buf;
        buf << uid << "." << icao << ".osgearth_pseudo_amg";
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
        supportsExtension( "osgearth_pseudo_amg", "Aerodrome model pseudo-loader" );
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

        osg::ref_ptr<AerodromeModelGraph> graph = getGraph(uid);
        if ( graph.valid() )
        {
            // Take a reference on the map to avoid map destruction during thread operation
            osg::ref_ptr<const Map> map = graph->getMap();
            if (map.valid() == true)
            {
                AerodromeNode* node = graph->getAerodromeNode(std::string(icao));
                if (node)
                {
                    Registry::instance()->startActivity(uri);

                    // create an AerodromeRenderer (node visitor) to create the geometry
                    osgEarth::Aerodrome::AerodromeRenderer renderer(map.get(), options);
                    node->accept(renderer);

                    Registry::instance()->endActivity(uri);
                    return ReadResult(node);
                }
            }
        }

        return ReadResult::ERROR_IN_READING_FILE;
    }

    static UID registerGraph( AerodromeModelGraph* graph )
    {
        Threading::ScopedWriteLock lock( _amgMutex );
        UID key = ++_uid;
        _amgRegistry[key] = graph;
        OE_DEBUG << "Registered AMG " << key << std::endl;
        return key;
    }

    static void unregisterGraph( UID uid )
    {
        Threading::ScopedWriteLock lock( _amgMutex );
        _amgRegistry.erase( uid );
        OE_DEBUG << "Unregistered AMG " << uid << std::endl;
    }

    static AerodromeModelGraph* getGraph( UID uid ) 
    {
        Threading::ScopedReadLock lock( _amgMutex );
        AMGRegistry::const_iterator i = _amgRegistry.find( uid );
        return i != _amgRegistry.end() ? i->second.get() : 0L;
    }
};

REGISTER_OSGPLUGIN(osgearth_pseudo_amg, osgEarthAerodromeModelPseudoLoader);


AerodromeModelGraph::AerodromeModelGraph(const Map* map)
  : _map(map)
{
    _uid = osgEarthAerodromeModelPseudoLoader::registerGraph( this );
}

AerodromeModelGraph::~AerodromeModelGraph()
{
    osgEarthAerodromeModelPseudoLoader::unregisterGraph( _uid );
}

AerodromeNode*
AerodromeModelGraph::getOrCreateAerodromeNode(const std::string& icao)
{
    AerodromeNode* an = 0L;

    AerodromeNodeMap::iterator itr = _aerodromes.find(icao);
    if (itr != _aerodromes.end())
    {
        an = itr->second.get();
    }
    else
    {
        an = new AerodromeNode(icao);
        _aerodromes[icao] = an;

        OE_NOTICE << LC << "Added new aerodrome: " << icao << std::endl;
    }

    return an;
}

AerodromeNode*
AerodromeModelGraph::getAerodromeNode(const std::string& icao)
{
    AerodromeNodeMap::iterator itr = _aerodromes.find(icao);
    if (itr != _aerodromes.end())
        return itr->second.get();

    return 0L;
}

int
AerodromeModelGraph::setupPaging()
{
    removeChildren(0, getNumChildren());

    int count = 0;
    for (AerodromeNodeMap::const_iterator it = _aerodromes.begin(); it != _aerodromes.end(); ++it)
    {
        AerodromeNode* an = it->second.get();
        if (an)
        {
            std::string uri = s_makeURI( _uid, an->icao() );

            //TODO: find better max range and make configurable
            float maxRange = 100000.0f;

            if (an->getBoundary() && an->getBoundary()->getFeature()->getGeometry())
            {
                osg::PagedLOD* p = new osg::PagedLOD();
                p->setFileName(0, uri);

                GeoPoint gp(an->getBoundary()->getFeature()->getSRS(), an->getBoundary()->getFeature()->getGeometry()->getBounds().center());
                osg::Vec3d center;
                gp.toWorld(center);
                p->setCenter(center);

                p->setRadius(std::max((float)an->getBoundary()->getFeature()->getGeometry()->getBounds().radius(), maxRange));
                p->setRange(0, 0.0f, maxRange);

                addChild(p);
            }
            else if (an->bounds().valid())
            {
                osg::PagedLOD* p = new osg::PagedLOD();
                p->setFileName(0, uri);

                GeoPoint gp(_map->getSRS(), an->bounds().center());
                osg::Vec3d center;
                gp.toWorld(center);
                p->setCenter(center);

                p->setRadius(std::max((float)an->bounds().radius(), maxRange));
                p->setRange(0, 0.0f, maxRange);

                addChild(p);
            }
            else
            {
                OE_WARN << LC << "Unable to create PagedLOD for aerodrome " << an->icao() << std::endl;
            }
        }

        count++;
    }


    return count;
}