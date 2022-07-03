import requests
controlData = requests.get('http://192.168.1.13:3000/controls')
print(controlData.json())
print(type(controlData.json()))
