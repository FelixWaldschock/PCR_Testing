from influxdb import InfluxDBClient
from datetime import datetime

#Setup database
client = InfluxDBClient(host='localhost', port=8086)
#client.create_database('mydb')
client.get_list_database()
client.switch_database('PCR_Testing')

#Setup Payload

def getPayload(valueDict):

    data = {
    "measurement": "PCR_TestCycling",
    "tags": {
        "ticker": "Test Number" 
        },
    "time": datetime.now(),
    "fields": valueDict
    }
    return data

def send2DB(valueDict):
    json_payload = []
    data = getPayload(valueDict)
    json_payload.append(data)
    #print(json_payload)
    client.write_points(json_payload)
    return True
