var exec = require('child_process').exec;

exec('node compile.js',
    function (error, stdout, stderr) {
        console.log('stdout: ' + stdout);
    });

exec('gazebo --verbose ./gazebo/worlds/main.world',
    function (error, stdout, stderr) {
        console.log('stdout: ' + stdout);
    });