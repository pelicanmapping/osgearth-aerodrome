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

#include "AerodromeModelSource"
#include <osgEarthAerodrome/AerodromeFactory>
#include <osgEarthAerodrome/AerodromeRenderer>
#include <osgEarth/ModelSource>
#include <osgEarth/Registry>


using namespace osgEarth;
using namespace osgEarth::Aerodrome;
using namespace osgEarth::Drivers::Aerodrome;


#define LC "[AerodromeModelSource] "

//------------------------------------------------------------------------

AerodromeModelSource::AerodromeModelSource( const ModelSourceOptions& options ) :
ModelSource( options ),
_options   ( options )
{
    //nop
}

void 
AerodromeModelSource::initialize(const osgDB::Options* dbOptions)
{
    ModelSource::initialize( dbOptions );

    _dbOptions = dbOptions;

    std::string uri = _options.getConfig().value("url");
    if ( !uri.empty() )
    {
        _catalog = AerodromeCatalog::read(uri, dbOptions);
    }
    else
    {
        _catalog = new AerodromeCatalog();
        _catalog->fromConfig(_options.getConfig());
    }
}

osg::Node*
AerodromeModelSource::createNodeImplementation(const Map* map, ProgressCallback* progress )
{
    AerodromeFactory factory(map);
    osg::ref_ptr<osg::Node> node = factory.createAerodromes(_catalog.get(), _dbOptions);

    return node.release();
}

//------------------------------------------------------------------------

class AerodromeModelSourceFactory : public ModelSourceDriver
{
public:
    AerodromeModelSourceFactory()
    {
        supportsExtension( "osgearth_model_aerodrome", "osgEarth aerodrome model plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Aerodrome Model Plugin";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new AerodromeModelSource( getModelSourceOptions(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_model_aerodrome, AerodromeModelSourceFactory)
