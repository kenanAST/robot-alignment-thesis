const util = require('util');
const exec = util.promisify(require('child_process').exec);
const fs = require('fs');

const files = fs.readdirSync('./ros2_ws');


let ROS2_WS = {
    repositories: [],
}

for(let i = 0; i<files.length; i++){
    ROS2_WS.repositories.push({name: files[i], packages: []});
}

ROS2_WS.repositories.map((element) => {
    element.packages = fs.readdirSync(`./ros2_ws/${element.name}/src`)
})

ROS2_WS.repositories.map( async (element) => {
    
    exec(`cd ./ros2_ws/${element.name}`,
        function (error, stdout, stderr) {
            console.log('stdout: ' + stdout);
        });

    exec('. install/setup.bash',
        function (error, stdout, stderr) {
            console.log('stdout: ' + stdout);
        });
    
    for(let i = 0; i<element.packages.length; i++){
        exec(`colcon build --packages-select ${element.packages[i]}`,
            function (error, stdout, stderr) {
                console.log('stdout: ' + stdout);
            });
    }
})


// exec('cd robovolc_ws',
//     function (error, stdout, stderr) {
//         console.log('stdout: ' + stdout);
//     });

// exec('. install/setup.bash',
//     function (error, stdout, stderr) {
//         console.log('stdout: ' + stdout);
//     });

// exec('colcon build --packages-select robov',
//     function (error, stdout, stderr) {
//         console.log('stdout: ' + stdout);
//     });