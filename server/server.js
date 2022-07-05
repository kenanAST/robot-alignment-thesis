const express = require('express');
const fs = require('fs');

const app = express();
const port = 3001;

app.use(express.json());
app.use(express.urlencoded({ extended: false }));

app.get('/controls', (req, res) => {
	const data = fs.readFileSync('./control.json', {
		encoding: 'utf8',
		flag: 'r',
	});

	res.send(JSON.parse(data));
});

app.get('/', (req, res) => {
	res.send('Mytest');
});

app.post('/controls', (req, res) => {
	const data = req.body;
	fs.writeFileSync('./control.json', JSON.stringify(data));
	res.send(data);
});

app.listen(port, () => {
	console.log(`Example app listening on port ${port}`);
});
