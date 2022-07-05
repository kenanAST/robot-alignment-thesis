import requests
controlData = requests.get('http://localhost:3000/controls')
print(controlData.json())
print(type(controlData.json()))
