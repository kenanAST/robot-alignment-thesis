var exec = require('child_process').exec;

exec('node compile.js',
    function (error, stdout, stderr) {
        console.log('stdout: ' + stdout);
    });

exec('gz sim -v 4 ./gazebo/worlds/main.world',
    function (error, stdout, stderr) {
        console.log('stdout: ' + stdout);
    });