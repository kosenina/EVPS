## Util scripts

Script purge.py is used to delete XML elements with attribute "deleted". 
This script comes in handy when editing SUMO map file using JOSM. 
If you delete something in the map file using JOSM it is not deleted in the XML file of the map, but the program just appends atribute "deleted". 
This attribute is not compatible with SUMO and you need to remove XML elements if you want to get rid of some parts of the map. 
To do so use script purge.py.