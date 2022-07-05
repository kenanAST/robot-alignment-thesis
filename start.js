var exec = require('child_process').exec;

exec('gazebo --verbose ./gazebo/worlds/main.world',
    function (error, stdout, stderr) {
        console.log('stdout: ' + stdout);
    });