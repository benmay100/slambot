import os
from ament_index_python.packages import get_package_share_directory
import json 

# 1. Get the path to the installed package's share directory
package_name = 'slambot_controllers'
share_directory = get_package_share_directory(package_name)

# 2. Construct the full path to the JSON file
json_file_path = os.path.join(share_directory, 'data', 'qr_codes_coordinates.json')

# 3. Read the data
with open(json_file_path, 'r') as f:
    qr_waypoints = json.load(f)

# The variable qr_waypoints now holds your dictionary!