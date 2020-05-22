/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#define USAGE "\nUSAGE: map_server <map_config.yaml>\n \
                  map_config.yaml:\n \
                    directory: <where-map-files>\n \
                    maps:\n \
                      -<mapname_1>.yaml \n \
                      -<mapname_2>.yaml \n \
                      ... \n"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <map>
#include <boost/filesystem.hpp>
#include <unistd.h>
#include <pwd.h>
#include <sys/types.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "multi_map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"
#include "std_srvs/Empty.h"
#include "multi_map_server/MapLoad.h"

using namespace std;

typedef nav_msgs::GetMap::Response MapData;
typedef nav_msgs::MapMetaData MapMetaData;
typedef map<string, MapData> MapDict;

vector<string> tokenize_getline(const string& data, const char delimiter) {
	vector<string> result;
	string token;
	stringstream ss(data);

	while (getline(ss, token, delimiter)) {
		result.push_back(token);
	}
	return result;
}

#ifdef HAVE_YAMLCPP_GT_0_5_0
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class MapServer
{
  public:
    /** Trivial constructor */
    MapServer(const string& fname) : directory("~/.ros/")
    {
      file_to_cache(fname);

      srv_load = n.advertiseService("/map_server/load_map", &MapServer::loadCallback, this);
	  srv_static = n.advertiseService("static_map", &MapServer::mapCallback, this);

      // Latched publisher for metadata
      metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
	  // Latched publisher for data
      map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    }

    bool file_to_cache(const string& fname)
    {
      // read yaml file
      ifstream fin(fname.c_str());

      if (fin.fail()) {
        ROS_ERROR("Map_server could not open %s.", fname.c_str());
        exit(-1);
      }
#ifdef HAVE_YAMLCPP_GT_0_5_0
      // The document loading process changed in yaml-cpp 0.5.
      YAML::Node doc = YAML::Load(fin);
#else
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
#endif

      // get directory where map files are
      try
      {
        doc["directory"] >> directory;
      }
      catch( YAML::Exception& e )
      {
        ROS_WARN("Could not read \'directory\' from YAML : %s",e.what());
        exit(-1);
      }

	  if(directory[0] == '~')
	  {
		  directory.erase(0,1);
		  directory = string( getenv("HOME") ) + directory;
	  }

      if(directory.back() != '/')
        directory.push_back('/');
      
      ROS_INFO("Map directory : %s",directory.c_str());

      // read map files and cache them
      try
      {
        for(YAML::const_iterator it = doc["maps"].begin(); it != doc["maps"].end(); ++it )
        {
          string mapfname = directory + it->as<string>();

          ROS_INFO("Reading map file from [%s]",mapfname.c_str());

          read_file(mapfname);
        }
      }
      catch( YAML::Exception& e )
      {
        ROS_WARN("Error on reading \'maps\' list from YAML : %s",e.what());
        exit(-1);
      }
    }

    bool read_file(const string& fname)
    {
      string mapfname = "";
      double origin[3];
      int negate;
      double occ_th, free_th;
      MapMode mode = TRINARY;
      ros::NodeHandle private_nh("~");
      private_nh.param("frame_id", frame_id, string("map"));

      ifstream fin(fname.c_str());
      if (fin.fail()) {
        ROS_ERROR("Map_server could not open %s.", fname.c_str());
        return false;
      }
#ifdef HAVE_YAMLCPP_GT_0_5_0
      // The document loading process changed in yaml-cpp 0.5.
      YAML::Node doc = YAML::Load(fin);
#else
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
#endif
      try {
        doc["resolution"] >> resolution;
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
        return false;
      }
      try {
        doc["negate"] >> negate;
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a negate tag or it is invalid.");
        return false;
      }
      try {
        doc["occupied_thresh"] >> occ_th;
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
        return false;
      }
      try {
        doc["free_thresh"] >> free_th;
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
        return false;
      }
      try {
        string modeS = "";
        doc["mode"] >> modeS;

        if(modeS=="trinary")
          mode = TRINARY;
        else if(modeS=="scale")
          mode = SCALE;
        else if(modeS=="raw")
          mode = RAW;
        else{
          ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
          return false;
        }
      } catch (YAML::Exception &) {
        ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
        mode = TRINARY;
      }
      try {
        doc["origin"][0] >> origin[0];
        doc["origin"][1] >> origin[1];
        doc["origin"][2] >> origin[2];
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain an origin tag or it is invalid.");
        return false;
      }
      try {
        doc["image"] >> mapfname;
        // TODO: make this path-handling more robust
        if(mapfname.size() == 0)
        {
          ROS_ERROR("The image tag cannot be an empty string.");
          return false;
        }

        boost::filesystem::path mapfpath(mapfname);
        if (!mapfpath.is_absolute())
        {
          boost::filesystem::path dir(fname);
          dir = dir.parent_path();
          mapfpath = dir / mapfpath;
          mapfname = mapfpath.string();
        }
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        return false;
        }
    
      ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());

      try
      {
          MapData tmp;

          multi_map_server::loadMapFromFile(&tmp, mapfname.c_str(),resolution,negate,occ_th,free_th, origin, mode);

          vector<string> tokens_dir = tokenize_getline(mapfname, '/');
		  vector<string> tokens_name = tokenize_getline(tokens_dir.back(), '.');

          // Save map name
		  current_map = tokens_name[0];

          // Add new map data type to dict
          mapdict.insert(std::make_pair(current_map, tmp));
      }
      catch (std::runtime_error e)
      {
          ROS_ERROR("%s", e.what());
          return false;
      }

      ROS_INFO("Map file \"%s\" is cached as map name [%s]", mapfname.c_str(),current_map.c_str());
      return true;
      
    }

    bool load(const string& mapname)
    {
      // Could not find map --> try to read |--> failed --> return
      //                                    |--> succeeded --> publish map
      if( mapdict.find(mapname) == mapdict.end() )
      {
        ROS_WARN("Could not find map name : %s",mapname.c_str());

        string mapfname = directory + mapname + string(".yaml");
        
        ROS_INFO("Trying to read file [%s]",mapfname.c_str());
        if( !read_file(mapfname) )
        {
          ROS_ERROR("Failed to read file");
          return false;
        }
        else{
          ROS_INFO("Succeeded to read file");
		  return true;
        }
		return false;	
		
      }

      
      // To make sure get a consistent time in simulation
      ros::Time::waitForValid();
      mapdict[mapname].map.info.map_load_time = ros::Time::now();
      mapdict[mapname].map.header.frame_id = frame_id;
      mapdict[mapname].map.header.stamp = ros::Time::now();

      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               mapdict[mapname].map.info.width,
               mapdict[mapname].map.info.height,
               mapdict[mapname].map.info.resolution);

	  MapMetaData meta_data_message_;
      meta_data_message_ = mapdict[mapname].map.info;

      metadata_pub.publish( meta_data_message_ );
      map_pub.publish( mapdict[mapname].map );

      // save current map name
      current_map = mapname;

	  return true;
    }
    

  private:
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    ros::ServiceServer srv_static, srv_load;
    bool deprecated;

    bool loadCallback(multi_map_server::MapLoad::Request &req,
					  multi_map_server::MapLoad::Response &res )
    {
      ROS_INFO("Loading map [%s]",req.data.c_str());
      res.succeeded = load(req.data);
	  return true;
    }

    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      try{
        res = mapdict[current_map];
      }
      catch(std::runtime_error e){
        ROS_ERROR("%s", e.what());
        return false;
      }
    
      ROS_INFO("Sending map");

      return true;
    }

    /** The map data is cached here, to be sent out to service callers
     */
    MapDict mapdict;
    string current_map;
    string directory;
	string frame_id;
	double resolution;

    /*
    void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
    {
      pub.publish( meta_data_message_ );
    }
    */

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_map_server");
  ROS_INFO("Map Server Started");

  if(argc != 2)
  {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }
  try
  {
    MapServer ms(argv[1]);
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}

