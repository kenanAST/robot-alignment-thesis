var fs = require('fs');
var files = fs.readdirSync('./ros2_ws');


let ROS2_WS = {
    repositories: [],
}

for(let i = 0; i<files.length; i++){
    ROS2_WS.repositories.push({name: files[i], packages: []});
}

ROS2_WS.repositories.map((element) => {
    element.packages = fs.readdirSync(`./ros2_ws/${element.name}/src`)
})
