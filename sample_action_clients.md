ros2 action send_goal /uav_1/move_drone custom_interfaces/action/MoveDrone "{command: 4, geo_points: [{lat: 47.398214, lon: 8.546392, alt: 10.0},{lat: 47.397621, lon: 8.546518, alt: 10.0},{lat: 47.398047, lon: 8.545744, alt: 10.0},{lat: 47.397338, lon: 8.545986, alt: 10.0},{lat: 47.398401, lon: 8.545921, alt: 10.0}], formation: {formation_type: 0, space: 2.0}}"

ros2 action send_goal /uav_1/move_drone custom_interfaces/action/MoveDrone "{command: 1}"

ros2 action send_goal /uav_1/move_drone custom_interfaces/action/MoveDrone "{command: 3, geo_points: [{lat: 47.398040, lon: 8.545950, alt: 10.0},{lat: 47.398040, lon: 8.546190, alt: 10.0},{lat: 47.397920, lon: 8.546190, alt: 10.0},{lat: 47.397920, lon: 8.545950, alt: 10.0}], formation: {formation_type: 0, space: 2.0}}"

ros2 action send_goal /uav_1/move_drone custom_interfaces/action/MoveDrone "{command: 3, geo_points: [{lat: 47.398150, lon: 8.545930, alt: 10.0},{lat: 47.398260, lon: 8.546070, alt: 10.0},{lat: 47.398190, lon: 8.546260, alt: 10.0},{lat: 47.398000, lon: 8.546290, alt: 10.0},{lat: 47.397870, lon: 8.546120, alt: 10.0},{lat: 47.397940, lon: 8.545920, alt: 10.0}], formation: {formation_type: 0, space: 1.5}}"