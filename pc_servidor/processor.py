#%%
import pymongo as mongo

#connect to the database SistCiber and the collection Accel
db = mongo.MongoClient("mongodb://localhost:27017/")["SistCiber"]
collection = db["Accel"]
#find all the documents in the collection
documents = collection.find()
data_64 = []
data_1024 = []

for document in documents:
    #save each document in a different list
    #remove the _id field
    document.pop("_id")
    if document["muestras"] == "64":
        document.pop("muestras")
        data_64.append(document)
    else:
        document.pop("muestras")
        data_1024.append(document)
# %%
#iterate each list
#%%
import numpy as np
import matplotlib.pyplot as plt
#iterate each list
for i in range(len(data_64)):
    #iterate each document
    x_accel = []
    y_accel = []
    z_accel = []
    for j in range(len(data_64[i])):
        #iterate each sample
        for k in range(len(data_64[i][str(j+1)])):
            print(data_64[i][str(j+1)][" x_accel: "])
            x_accel.append(data_64[i][str(j+1)][" x_accel: "])
            y_accel.append(data_64[i][str(j+1)][" y_accel: "])
            z_accel.append(data_64[i][str(j+1)][" z_accel: "])
    #do a fft of x_accel
    x_fft = np.fft.fft(x_accel)
    #plot the fft
    plt.plot(x_fft)
    plt.show()
    
        
# %%
#show the x_fft in graphana
import requests
import json
import time

#iterate each list
for i in range(len(data_64)):
    #iterate each document
    x_accel = []
    y_accel = []
    z_accel = []
    for j in range(len(data_64[i])):
        #iterate each sample
        for k in range(len(data_64[i][str(j+1)])):
            print(data_64[i][str(j+1)][" x_accel: "])
            x_accel.append(data_64[i][str(j+1)][" x_accel: "])
            y_accel.append(data_64[i][str(j+1)][" y_accel: "])
            z_accel.append(data_64[i][str(j+1)][" z_accel: "])
    #do a fft of x_accel
    x_fft = np.fft.fft(x_accel)
    #conver x_fft to a type that json can serialize
    x_fft = x_fft.real
    #plot the fft
    plt.plot(x_fft)
    plt.show()

    url = "http://admin:admin@localhost:3000/api/dashboards/db"
    payload = {
        "dashboard": {
            "id": None,
            "uid": None,
            "title": "FFT",
            "tags": [],
            "timezone": "browser",
            "schemaVersion": 16,
            "version": 0
        },
        "overwrite": False
    }
    headers = {
        'Content-Type': 'application/json'
    }
    response = requests.request("POST", url, headers=headers, data = json.dumps(payload))
    print(response.text.encode('utf8'))

    #send the fft to grafana
    # url = "http://admin:admin@localhost:3000/api/datasources/proxy/1/api/v1/push"
    # headers = {
    #     "Content-Type": "application/json"
    # }
    # data = {
    #     "series": [
    #         {
    #             "name": "x_fft",
    #             "columns": ["time", "value"],
    #             "points": []
    #         }
    #     ]
    # }
    # for i in range(len(x_fft)):
    #     data["series"][0]["points"].append([i, x_fft[i]])
    # data = json.dumps(data)
    # response = requests.post(url, headers=headers, data=data)
    # print(response.text)
    time.sleep(5)
# %%
#create a new panel in the dashboard that show the fft
url = "http://admin:admin@localhost:3000/api/dashboards/db"
payload = {
    "dashboard": {
        "id": None,
        "uid": None,
        "title": "FFT pajera",
        "tags": [],
        "timezone": "browser",
        "schemaVersion": 16,
        "version": 0,
        "panels": [
            {
                "datasource": None,
                "fieldConfig": {
                    "defaults": {
                        "custom": {}
                    },
                    "overrides": []
                },
                "gridPos": {
                    "h": 8,
                    "w": 12,
                    "x": 0,
                    "y": 0
                },
                "id": 2,
                "options": {
                    "legend": {
                        "calcs": [],
                        "displayMode": "list",
                        "placement": "bottom"
                    },
                    "tooltip": {
                        "mode": "single"
                    }
                },
                "pluginVersion": "7.1.1",
                "targets": [
                    {
                        "refId": "A",
                        "scenarioId": "random_walk"
                    }
                ],
                "title": "FFT pajera",
                "type": "timeseries"
            }
        ]
    },
    "folderId": 0,
    "overwrite": False
}
headers = {
    'Content-Type': 'application/json'
}
response = requests.request("POST", url, headers=headers, data = json.dumps(payload))
print(response.text.encode('utf8'))

# %%
