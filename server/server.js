const express = require('express');
const fs = require('fs');

const app = express()
const port = 3000

const data = fs.readFileSync('./control.json',
            {encoding:'utf8', flag:'r'});

app.get('/controls', (req, res) => {
  res.send(JSON.parse(data))
})

app.get('/', (req, res) => {
    res.send('Mytest');
})

app.listen(port, () => {
  console.log(`Example app listening on port ${port}`)
})



