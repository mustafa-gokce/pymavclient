import pprint
import pymavclient

client = pymavclient.PyMAVClient()
client.vehicle_update(fresh=True)
pprint.pprint(client.vehicle.__dict__())
