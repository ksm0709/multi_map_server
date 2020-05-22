# multi_map_server
This map server uploads multiple map on the memory. And support service interface for loading map.  
(Copied &amp; modified from https://github.com/ros-planning/navigation.git )

- How to run
'''
roslaunch multi_map_server server.launch
'''

- How to add / remove map
you can edit your map list by editing 'cfg/map_config.yaml' file.
(OR you can use any yaml file with same format of 'cfg/map_config.yaml')

and also you can just request map through '/map_server/load_map' interface,
server will try to find map file in the 'directory'( written in yaml file )
