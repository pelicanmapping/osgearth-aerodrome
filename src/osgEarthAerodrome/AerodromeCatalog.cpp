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

#include "AerodromeCatalog"
#include "AerodromeFeatureOptions"
#include <osgEarth/Config>
#include <osgEarth/XmlUtils>

using namespace osgEarth;
using namespace osgEarth::Aerodrome;

#define LC "[AerodromeCatalog] "

#define AERODROME_CATALOG_CURRENT_VERSION 1


AerodromeCatalog::AerodromeCatalog()
{
    _version = AERODROME_CATALOG_CURRENT_VERSION;
}

void
AerodromeCatalog::fromConfig(const Config& conf)
{
    conf.getIfSet("version", _version);

    if ( conf.hasChild("boundaries") )
        _boundaryOptions->merge( conf.child("boundaries") );

    if ( conf.hasChild("light_beacons") )
        _lightBeaconOptions->merge( conf.child("light_beacons") );

    if ( conf.hasChild("light_indicators") )
        _lightIndicatorOptions->merge( conf.child("light_indicators") );

    if ( conf.hasChild("linear_features") )
        _linearFeatureOptions->merge( conf.child("linear_features") );

    if ( conf.hasChild("pavement") )
        _pavementOptions->merge( conf.child("pavement") );

    if ( conf.hasChild("runways") )
        _runwayOptions->merge( conf.child("runways") );

    if ( conf.hasChild("runway_thresholds") )
        _runwayThresholdOptions->merge( conf.child("runway_thresholds") );

    if ( conf.hasChild("startup_locations") )
        _startupLocationOptions->merge( conf.child("startup_locations") );

    if ( conf.hasChild("stopways") )
        _stopwayOptions->merge( conf.child("stopways") );

    if ( conf.hasChild("taxiways") )
        _taxiwayOptions->merge( conf.child("taxiways") );

    if ( conf.hasChild("terminals") )
        _terminalOptions->merge( conf.child("terminals") );

    if ( conf.hasChild("windsocks") )
        _windsockOptions->merge( conf.child("windsocks") );

}

Config
AerodromeCatalog::getConfig() const
{
    Config conf;
    conf.addIfSet("version", _version);
    conf.updateObjIfSet( "boundaries", _boundaryOptions );
    conf.updateObjIfSet( "light_beacons", _lightBeaconOptions );
    conf.updateObjIfSet( "light_indicators", _lightIndicatorOptions );
    conf.updateObjIfSet( "linear_features", _linearFeatureOptions );
    conf.updateObjIfSet( "pavement", _pavementOptions );
    conf.updateObjIfSet( "runways", _runwayOptions );
    conf.updateObjIfSet( "runway_thresholds", _runwayThresholdOptions );
    conf.updateObjIfSet( "startup_locations", _startupLocationOptions );
    conf.updateObjIfSet( "stopways", _stopwayOptions );
    conf.updateObjIfSet( "taxiways", _taxiwayOptions );
    conf.updateObjIfSet( "terminals", _terminalOptions );
    conf.updateObjIfSet( "windsocks", _windsockOptions );

    return conf;
}

AerodromeCatalog*
AerodromeCatalog::read(const URI& uri, const osgDB::Options* options)
{
    osg::ref_ptr<AerodromeCatalog> catalog;

    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( uri, options );
    if ( doc.valid() )
    {
        catalog = new AerodromeCatalog();
        catalog->fromConfig( doc->getConfig().child("catalog") );
    }
    else
    {
        OE_WARN << LC << "Failed to read catalog from " << uri.full() << "\n";
    }

    return catalog.release();
}