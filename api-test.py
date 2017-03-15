import requests
import json

URL = "https://lkdnb0wacl.execute-api.us-west-2.amazonaws.com/initial/myresource"

payload = {
  "position": [1,2],
  "velocity": [3,4],
  "coverage": 0.5,
  "role": "worker"
}

r = requests.post(URL, data=json.dumps(payload))

print(r.status_code)
print(r.json())
"""
Expect: {
    "position": [4, 6],
    "coverage": 0.5,
    "role": "worker"
}
"""

