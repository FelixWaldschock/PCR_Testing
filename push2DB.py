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
    "measurement": "CT-Values",
    "tags": {
        "ticker": "Test Number" 
        },
    "time": datetime.now(),
    "fields": {
        'Temperature': valueDict["temperature"],
        'Virus_intensity': f,
        'close': 667.93
        }
    }
    return data

def send2DB(valueDict):
    json_payload = []
    data = getPayload(valueDict)
    json_payload.append(data)
    return True
